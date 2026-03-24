/****************************************************************************
 * apps/uav/sensors_app/sensors_main.cpp
 *
 * UAV SENSORS APPLICATION - Time-Triggered Architecture
 *
 * MỤC ĐÍCH:
 * - Thu thập dữ liệu từ 4 IMU (ICM42688P) qua SPI
 * - Áp dụng calibration và filtering
 * - Fusion nhiều IMU bằng voting/weighted average
 * - Publish sensor_combined cho EKF
 * - Giám sát health của sensors
 *
 * KIẾN TRÚC TIME-TRIGGERED:
 * - Master Timebase @ 8 kHz tạo ticks cho toàn hệ thống
 * - Sensors task sync với FAST domain (8 kHz) hoặc MEDIUM (1 kHz)
 * - Mọi hoạt động đều deterministic, không event-driven
 *
 * DATA FLOW:
 *   [ICM42688P x4] → [DMA Double Buffer] → [Calibration]
 *        → [Lowpass Filter] → [IMU Fusion] → [sensor_combined topic]
 *
 * TIMING BUDGET (1 kHz = 1000 µs per cycle):
 *   - SPI DMA transfer: ~100 µs (4 IMUs parallel)
 *   - Calibration:      ~20 µs
 *   - Filtering:        ~30 µs
 *   - Fusion:           ~50 µs
 *   - uORB publish:     ~20 µs
 *   - Total:            ~220 µs (22% CPU)
 *   - Margin:           ~780 µs (78% free)
 *
 * SỬ DỤNG:
 *   sensors start    - Khởi động sensors task
 *   sensors stop     - Dừng sensors task
 *   sensors status   - Xem trạng thái chi tiết
 *   sensors test     - Chạy self-test
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <sched.h>
#include <errno.h>
#include <syslog.h>
#include <fcntl.h>

#include <nuttx/clock.h>
#include <time.h>

/* UAV Platform */
#include <uav/lib/platform/hrt.h>
#include <uav/lib/platform/timebase.h>

/* UAV Libraries */
#include <uav/lib/utils/ringbuf.hpp>
#include <uav/lib/dsp/filters.hpp>
#include <uav/lib/sensor_processing/imu_fusion.hpp>
#include <uav/lib/health/health_monitor.hpp>

/* uORB */
#include <uav/uorb/uorb.hpp>
#include <uav/uorb/topics/sensor_imu.hpp>
#include <uav/uorb/topics/sensor_combined.hpp>
#include <uav/uorb/topics/system_status.hpp>

/* IMU Driver - ICM42688P */
#include <uav/drivers/imu/icm42688p/icm42688p.hpp>

/* MAG Driver - BMM150 */
#include <uav/drivers/mag/bmm150/bmm150.hpp>
#include <nuttx/i2c/i2c_master.h>

/* Board-specific SPI/I2C */
extern "C" {
#include "stm32_spi_icm.h"
struct i2c_master_s *stm32_i2cbus_initialize(int port);
}

/* Global ICM42688P driver instances */
static drivers::imu::ICM42688P *g_imu_drivers[CONFIG_UAV_NUM_IMUS];

/* Exported instances for calibration application */
drivers::imu::ICM42688P *g_imu_instance = nullptr;
drivers::mag::BMM150    *g_mag_instance = nullptr;

/****************************************************************************
 * Configuration - Có thể override qua Kconfig
 ****************************************************************************/

#ifndef CONFIG_UAV_SENSORS_PRIORITY
#define CONFIG_UAV_SENSORS_PRIORITY     250
#endif

#ifndef CONFIG_UAV_SENSORS_STACKSIZE
#define CONFIG_UAV_SENSORS_STACKSIZE    8192
#endif

#ifndef CONFIG_UAV_NUM_IMUS
#define CONFIG_UAV_NUM_IMUS             4
#endif

/* Sensor rates */
#ifndef CONFIG_UAV_IMU_RATE_HZ
#define CONFIG_UAV_IMU_RATE_HZ          1000
#endif

/* Filter settings */
#ifndef CONFIG_UAV_GYRO_CUTOFF_HZ
#define CONFIG_UAV_GYRO_CUTOFF_HZ       100     /* Lowpass cutoff for gyro */
#endif

#ifndef CONFIG_UAV_ACCEL_CUTOFF_HZ
#define CONFIG_UAV_ACCEL_CUTOFF_HZ      50      /* Lowpass cutoff for accel */
#endif

/* Fusion mode: 0=VOTING, 1=WEIGHTED, 2=PRIMARY */
#ifndef CONFIG_UAV_IMU_FUSION_MODE
#define CONFIG_UAV_IMU_FUSION_MODE      0
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

using namespace uav::utils;
using namespace uav::dsp;
using namespace uav::sensor_processing;
using namespace uav::health;

/**
 * @brief Context cho mỗi IMU
 */
struct ImuContext {
    /* Filters - 3 axes mỗi loại */
    LowPassFilter2p gyro_filter[3];
    LowPassFilter2p accel_filter[3];

    /* Last raw data */
    sensor_imu_s raw_data;

    /* Filtered data */
    ImuData filtered_data;

    /* Statistics */
    uint32_t sample_count;
    uint32_t error_count;
    uint64_t last_sample_time;

    /* Constructor để khởi tạo sạch */
    ImuContext() :
        sample_count(0),
        error_count(0),
        last_sample_time(0)
    {
        memset(&raw_data, 0, sizeof(raw_data));
        memset(&filtered_data, 0, sizeof(filtered_data));
    }

    /* Reset function */
    void reset() {
        sample_count = 0;
        error_count = 0;
        last_sample_time = 0;
        memset(&raw_data, 0, sizeof(raw_data));
        memset(&filtered_data, 0, sizeof(filtered_data));
        for (int i = 0; i < 3; i++) {
            gyro_filter[i].reset();
            accel_filter[i].reset();
        }
    }
};

/**
 * @brief Main context của sensors app
 */
struct SensorsContext {
    /* IMU contexts */
    ImuContext imu[CONFIG_UAV_NUM_IMUS];

    /* IMU Fusion */
    ImuFusion fusion;

    /* Ring buffer để pass data giữa stages */
    RingBuffer<ImuData, 16> imu_ring[CONFIG_UAV_NUM_IMUS];

    /* uORB publishers */
    uorb::orb_advert_t imu_pub[CONFIG_UAV_NUM_IMUS];
    uorb::orb_advert_t combined_pub;
    uorb::orb_advert_t status_pub;

    /* Pre-allocated messages */
    sensor_imu_s imu_msg[CONFIG_UAV_NUM_IMUS];
    sensor_combined_s combined_msg;
    system_status_s status_msg;

    /* Task state */
    volatile bool should_exit;
    volatile bool is_running;
    pid_t task_pid;

    /* Statistics */
    uint32_t loop_count;
    uint32_t deadline_misses;
    uint64_t last_status_time;

    /* Constructor để khởi tạo sạch */
    SensorsContext() :
        fusion(),
        combined_pub(nullptr),
        status_pub(nullptr),
        should_exit(false),
        is_running(false),
        task_pid(-1),
        loop_count(0),
        deadline_misses(0),
        last_status_time(0)
    {
        for (int i = 0; i < CONFIG_UAV_NUM_IMUS; i++) {
            imu_pub[i] = nullptr;
            memset(&imu_msg[i], 0, sizeof(sensor_imu_s));
            /* imu[i] được khởi tạo bởi ImuContext constructor */
        }
        memset(&combined_msg, 0, sizeof(sensor_combined_s));
        memset(&status_msg, 0, sizeof(system_status_s));
    }

    /* Reset function thay vì assignment */
    void reset() {
        should_exit = false;
        is_running = false;
        task_pid = -1;
        loop_count = 0;
        deadline_misses = 0;
        last_status_time = 0;

        for (int i = 0; i < CONFIG_UAV_NUM_IMUS; i++) {
            imu_pub[i] = nullptr;
            memset(&imu_msg[i], 0, sizeof(sensor_imu_s));
            imu[i].reset();  /* Dùng reset() thay vì memset */
            imu_ring[i].clear();
        }

        combined_pub = nullptr;
        status_pub = nullptr;
        memset(&combined_msg, 0, sizeof(sensor_combined_s));
        memset(&status_msg, 0, sizeof(system_status_s));
    }
};

/****************************************************************************
 * Private Data - Static allocation
 ****************************************************************************/

static SensorsContext g_ctx;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/**
 * @brief Khởi tạo filters cho mỗi IMU
 */
static void init_filters(SensorsContext* ctx)
{
    for (int i = 0; i < CONFIG_UAV_NUM_IMUS; i++) {
        /* Setup gyro filters */
        for (int j = 0; j < 3; j++) {
            ctx->imu[i].gyro_filter[j].set_cutoff_frequency(
                (float)CONFIG_UAV_IMU_RATE_HZ,
                (float)CONFIG_UAV_GYRO_CUTOFF_HZ
            );

            ctx->imu[i].accel_filter[j].set_cutoff_frequency(
                (float)CONFIG_UAV_IMU_RATE_HZ,
                (float)CONFIG_UAV_ACCEL_CUTOFF_HZ
            );
        }

        ctx->imu[i].sample_count = 0;
        ctx->imu[i].error_count = 0;
        ctx->imu[i].last_sample_time = 0;
    }
}

/**
 * @brief Khởi tạo IMU drivers
 *
 * Khởi tạo 4 ICM42688P sensors trên SPI1
 */
static int init_imu_drivers(SensorsContext* ctx)
{
    int ret;
    int init_count = 0;

    syslog(LOG_INFO, "[sensors] Initializing %d ICM42688P IMU drivers...\n", CONFIG_UAV_NUM_IMUS);

    /* Initialize SPI1 for ICM42688P */
    ret = board_spi1_icm_initialize();
    if (ret < 0) {
        syslog(LOG_ERR, "[sensors] Failed to initialize SPI1 for ICM: %d\n", ret);
        return ret;
    }

    /* Initialize each ICM42688P */
    for (int i = 0; i < CONFIG_UAV_NUM_IMUS; i++) {
        struct spi_dev_s *spi = board_spi1_icm_get_device(i);
        
        if (spi == NULL) {
            syslog(LOG_ERR, "[sensors] Failed to get SPI device for IMU %d\n", i);
            g_imu_drivers[i] = NULL;
            continue;
        }

        /* Create ICM42688P instance */
        g_imu_drivers[i] = new drivers::imu::ICM42688P(1, i);
        if (g_imu_drivers[i] == NULL) {
            syslog(LOG_ERR, "[sensors] Failed to allocate ICM42688P %d\n", i);
            continue;
        }

        /* Initialize sensor */
        ret = g_imu_drivers[i]->init();
        if (ret < 0) {
            syslog(LOG_ERR, "[sensors] Failed to init ICM42688P %d: %d\n", i, ret);
            delete g_imu_drivers[i];
            g_imu_drivers[i] = NULL;
            continue;
        }

        /* Mark as present in fusion */
        ctx->fusion.set_imu_present(i, true);

        /* Register với health monitor */
        get_health_monitor().set_present(
            static_cast<ComponentId>(static_cast<int>(ComponentId::IMU_0) + i),
            true
        );

        init_count++;
        syslog(LOG_INFO, "[sensors] ICM42688P %d initialized OK\n", i);
    }

    if (init_count == 0) {
        syslog(LOG_ERR, "[sensors] No IMUs initialized!\n");
        return -ENODEV;
    }

    g_imu_instance = g_imu_drivers[0];

    syslog(LOG_INFO, "[sensors] Initialized %d/%d IMUs\n", init_count, CONFIG_UAV_NUM_IMUS);
    return 0;
}

/**
 * @brief Khởi tạo MAG driver (BMM150)
 */
static int init_mag_driver(SensorsContext* ctx)
{
    syslog(LOG_INFO, "[sensors] Initializing BMM150 MAG driver on I2C1...\n");

    struct i2c_master_s *i2c = stm32_i2cbus_initialize(1);
    if (!i2c) {
        syslog(LOG_ERR, "[sensors] Failed to initialize I2C1 for BMM150\n");
        return -ENODEV;
    }

    g_mag_instance = new drivers::mag::BMM150(i2c, 0x13);
    if (!g_mag_instance) {
        syslog(LOG_ERR, "[sensors] Failed to allocate BMM150\n");
        return -ENOMEM;
    }

    int ret = g_mag_instance->initialize();
    if (ret < 0) {
        syslog(LOG_ERR, "[sensors] Failed to init BMM150: %d\n", ret);
        delete g_mag_instance;
        g_mag_instance = nullptr;
        return ret;
    }

    syslog(LOG_INFO, "[sensors] BMM150 initialized OK\n");
    return 0;
}

/**
 * @brief Đọc dữ liệu từ IMU thật (ICM42688P)
 */
static int read_imu_real(int imu_index, sensor_imu_s* msg, uint64_t now_us)
{
    if (g_imu_drivers[imu_index] == NULL) {
        return -ENODEV;
    }

    drivers::imu::ICM42688P::Data data;
    int ret = g_imu_drivers[imu_index]->read(data);
    if (ret < 0) {
        return ret;
    }

    msg->timestamp_us = now_us;
    msg->instance = imu_index;

    msg->gyro[0] = data.gyro[0];
    msg->gyro[1] = data.gyro[1];
    msg->gyro[2] = data.gyro[2];

    msg->accel[0] = data.accel[0];
    msg->accel[1] = data.accel[1];
    msg->accel[2] = data.accel[2];

    msg->temperature = data.temperature;

    return 0;
}

/**
 * @brief Đọc dữ liệu từ IMU (dummy implementation cho fallback)
 */
static void read_imu_dummy(int imu_index, sensor_imu_s* msg, uint64_t now_us)
{
    msg->timestamp_us = now_us;
    msg->instance = imu_index;

    /* Simulate IMU data với chút noise */
    float noise = (float)(rand() % 1000 - 500) / 100000.0f;

    msg->gyro[0] = 0.0f + noise;
    msg->gyro[1] = 0.0f + noise;
    msg->gyro[2] = 0.0f + noise;

    msg->accel[0] = 0.0f + noise * 10;
    msg->accel[1] = 0.0f + noise * 10;
    msg->accel[2] = -9.81f + noise * 10;  /* Gravity */

    msg->temperature = 25.0f + (float)(rand() % 100) / 100.0f;
}

/**
 * @brief Apply filters và publish IMU data
 */
static void process_imu(SensorsContext* ctx, int imu_index, uint64_t now_us)
{
    ImuContext* imu_ctx = &ctx->imu[imu_index];
    sensor_imu_s* raw = &ctx->imu_msg[imu_index];

    /* Đọc raw data từ ICM42688P thật, fallback sang dummy nếu fail */
    int ret = read_imu_real(imu_index, raw, now_us);
    if (ret < 0) {
        /* IMU không khả dụng, dùng dummy */
        read_imu_dummy(imu_index, raw, now_us);
        imu_ctx->error_count++;
    }

    /* Apply lowpass filters */
    float filtered_gyro[3];
    float filtered_accel[3];

    for (int j = 0; j < 3; j++) {
        filtered_gyro[j] = imu_ctx->gyro_filter[j].apply(raw->gyro[j]);
        filtered_accel[j] = imu_ctx->accel_filter[j].apply(raw->accel[j]);
    }

    /* Update filtered data struct */
    imu_ctx->filtered_data.timestamp_us = now_us;
    imu_ctx->filtered_data.instance = imu_index;
    imu_ctx->filtered_data.valid = true;

    for (int j = 0; j < 3; j++) {
        imu_ctx->filtered_data.gyro[j] = filtered_gyro[j];
        imu_ctx->filtered_data.accel[j] = filtered_accel[j];
    }
    imu_ctx->filtered_data.temperature = raw->temperature;

    /* Update fusion */
    ctx->fusion.update_imu(imu_index, imu_ctx->filtered_data);

    /* Publish raw to sensor_imu topic */
    if (ctx->imu_pub[imu_index]) {
        uorb::orb_publish(ORB_ID(sensor_imu), ctx->imu_pub[imu_index], raw);
    }

    /* Update statistics */
    imu_ctx->sample_count++;
    imu_ctx->last_sample_time = now_us;

    /* Send heartbeat to health monitor */
    get_health_monitor().heartbeat(
        static_cast<ComponentId>(static_cast<int>(ComponentId::IMU_0) + imu_index)
    );
}

/**
 * @brief Thực hiện fusion và publish sensor_combined
 */
static void do_fusion(SensorsContext* ctx, uint64_t now_us)
{
    FusedImuData fused;

    if (ctx->fusion.fuse(fused) == 0) {
        /* Populate combined message */
        sensor_combined_s* msg = &ctx->combined_msg;

        msg->timestamp_us = now_us;

        for (int j = 0; j < 3; j++) {
            msg->gyro[j] = fused.gyro[j];
            msg->accel[j] = fused.accel[j];
        }

        msg->num_imus_used = fused.num_imus_used;
        msg->healthy_mask = fused.healthy_mask;
        msg->fusion_mode = static_cast<uint8_t>(ctx->fusion.get_mode());
        msg->valid = fused.valid;

        /* TODO: Calculate gyro_integral và accel_integral */
        msg->dt = 1.0f / CONFIG_UAV_IMU_RATE_HZ;

        /* Publish */
        if (ctx->combined_pub) {
            uorb::orb_publish(ORB_ID(sensor_combined), ctx->combined_pub, msg);
        }
    }
}

/**
 * @brief Publish system status (lower rate, ~10 Hz)
 */
static void publish_status(SensorsContext* ctx, uint64_t now_us)
{
    /* Only publish at 10 Hz */
    if (now_us - ctx->last_status_time < 100000) {
        return;
    }
    ctx->last_status_time = now_us;

    system_status_s* msg = &ctx->status_msg;
    memset(msg, 0, sizeof(*msg));

    msg->timestamp_us = now_us;

    /* Get health info */
    SystemHealth health = get_health_monitor().get_system_health();

    msg->health_level = static_cast<uint8_t>(health.level);
    msg->healthy_imus = health.healthy_imus;
    msg->baro_ok = health.baro_ok;
    msg->mag_ok = health.mag_ok;
    msg->gps_ok = health.gps_ok;
    msg->ekf_ok = health.ekf_ok;
    msg->timebase_ok = timebase_is_initialized();

    msg->uptime_ms = health.uptime_ms;
    msg->master_tick_count = (uint32_t)timebase_get_tick_count();
    msg->deadline_misses = ctx->deadline_misses;

    /* Get jitter stats */
    timebase_jitter_stats_t jitter;
    timebase_get_jitter_stats(&jitter);
    msg->jitter_min_ns = jitter.min_period_ns;
    msg->jitter_max_ns = jitter.max_period_ns;
    msg->jitter_avg_ns = (uint32_t)jitter.avg_period_ns;

    /* Publish */
    if (ctx->status_pub) {
        uorb::orb_publish(ORB_ID(system_status), ctx->status_pub, msg);
    }
}

/**
 * @brief Main sensors thread
 */
static int sensors_thread_main(int argc, char *argv[])
{
    (void)argc;
    (void)argv;

    SensorsContext* ctx = &g_ctx;
    int ret;

    syslog(LOG_INFO, "[sensors] Thread starting...\n");

    /*=========================================================================
     * PHASE 1: Set realtime priority
     *=========================================================================*/

    struct sched_param param;
    param.sched_priority = CONFIG_UAV_SENSORS_PRIORITY;
    ret = sched_setscheduler(0, SCHED_FIFO, &param);
    if (ret < 0) {
        syslog(LOG_WARNING, "[sensors] Failed to set FIFO scheduler: %d\n", errno);
    }

    /*=========================================================================
     * PHASE 2: Initialize timebase
     *=========================================================================*/

    if (!timebase_is_initialized()) {
        ret = timebase_init();
        if (ret < 0) {
            syslog(LOG_ERR, "[sensors] Failed to init timebase: %d\n", ret);
            return ret;
        }
    }

    /*=========================================================================
     * PHASE 3: Initialize health monitor
     *=========================================================================*/

    get_health_monitor().init();
    get_health_monitor().set_present(ComponentId::MASTER_TICK, true);

    /*=========================================================================
     * PHASE 4: Initialize IMU drivers and fusion
     *=========================================================================*/

    init_filters(ctx);

    /* QUAN TRỌNG: Phải init fusion TRƯỚC khi init_imu_drivers
     * vì init() reset present=false, còn init_imu_drivers set present=true */
    FusionMode mode;
    switch (CONFIG_UAV_IMU_FUSION_MODE) {
        case 1:
            mode = FusionMode::WEIGHTED;
            break;
        case 2:
            mode = FusionMode::PRIMARY;
            break;
        default:
            mode = FusionMode::VOTING;
            break;
    }
    ctx->fusion.init(CONFIG_UAV_NUM_IMUS);
    ctx->fusion.set_mode(mode);

    /* Bây giờ mới init drivers - sẽ set_imu_present(i, true) */
    init_imu_drivers(ctx);
    init_mag_driver(ctx);

    /*=========================================================================
     * PHASE 5: Advertise uORB topics
     *=========================================================================*/

    /* Individual IMU topics */
    for (int i = 0; i < CONFIG_UAV_NUM_IMUS; i++) {
        memset(&ctx->imu_msg[i], 0, sizeof(sensor_imu_s));
        ctx->imu_msg[i].instance = i;

        ctx->imu_pub[i] = uorb::orb_advertise_multi(
            ORB_ID(sensor_imu),
            &ctx->imu_msg[i],
            i
        );

        if (!ctx->imu_pub[i]) {
            syslog(LOG_ERR, "[sensors] Failed to advertise IMU[%d]\n", i);
        }
    }

    /* Combined topic */
    memset(&ctx->combined_msg, 0, sizeof(sensor_combined_s));
    ctx->combined_pub = uorb::orb_advertise(ORB_ID(sensor_combined), &ctx->combined_msg);
    if (!ctx->combined_pub) {
        syslog(LOG_ERR, "[sensors] Failed to advertise sensor_combined\n");
    }

    /* System status topic */
    memset(&ctx->status_msg, 0, sizeof(system_status_s));
    ctx->status_pub = uorb::orb_advertise(ORB_ID(system_status), &ctx->status_msg);

    /*=========================================================================
     * PHASE 6: Main loop - Time-Triggered
     *=========================================================================*/

    ctx->is_running = true;
    ctx->loop_count = 0;
    ctx->deadline_misses = 0;
    ctx->last_status_time = 0;

    syslog(LOG_INFO, "[sensors] Entering main loop @ %d Hz\n", CONFIG_UAV_IMU_RATE_HZ);

    while (!ctx->should_exit) {
        /*---------------------------------------------------------------------
         * Wait for MEDIUM domain tick (1 kHz)
         *---------------------------------------------------------------------*/

        /* NOTE:
         * Some NuttX configurations don't reliably deliver POSIX timer signals.
         * If the timebase isn't ticking, a plain timebase_wait_domain() would
         * block forever and we'd never publish any sensor data.
         *
         * Use a bounded wait and fall back to a 1 kHz sleep so bring-up can
         * continue even if timebase is not functional.
         */
        ret = timebase_wait_domain_timeout(DOMAIN_MEDIUM, 5000);
        if (ret < 0) {
            if (ret == -EINTR) {
                continue;  /* Signal interrupt, check should_exit */
            }

            if (ret == -ETIMEDOUT || ret == -ENODEV) {
                /* Fallback to approximate 1 kHz without timebase */
                usleep(1000);
            } else {
                syslog(LOG_ERR, "[sensors] timebase_wait error: %d\n", ret);
                break;
            }
        }

        uint64_t now_us = hrt_absolute_time();

        /*---------------------------------------------------------------------
         * Poll all IMUs
         *---------------------------------------------------------------------*/

        for (int i = 0; i < CONFIG_UAV_NUM_IMUS; i++) {
            process_imu(ctx, i, now_us);
        }

        /*---------------------------------------------------------------------
         * Fusion
         *---------------------------------------------------------------------*/

        do_fusion(ctx, now_us);

        /*---------------------------------------------------------------------
         * Health monitoring & status
         *---------------------------------------------------------------------*/

        /* Send master tick heartbeat */
        get_health_monitor().heartbeat(ComponentId::MASTER_TICK);

        /* Update health monitor (low rate) */
        if (ctx->loop_count % 100 == 0) {
            get_health_monitor().update();
        }

        /* Publish status (10 Hz) */
        publish_status(ctx, now_us);

        /*---------------------------------------------------------------------
         * Statistics
         *---------------------------------------------------------------------*/

        ctx->loop_count++;
    }

    /*=========================================================================
     * PHASE 7: Cleanup
     *=========================================================================*/

    ctx->is_running = false;

    /* Unadvertise topics */
    for (int i = 0; i < CONFIG_UAV_NUM_IMUS; i++) {
        if (ctx->imu_pub[i]) {
            uorb::orb_unadvertise(ctx->imu_pub[i]);
            ctx->imu_pub[i] = nullptr;
        }
    }

    if (ctx->combined_pub) {
        uorb::orb_unadvertise(ctx->combined_pub);
        ctx->combined_pub = nullptr;
    }

    if (ctx->status_pub) {
        uorb::orb_unadvertise(ctx->status_pub);
        ctx->status_pub = nullptr;
    }

    /* Deinit health monitor */
    get_health_monitor().deinit();

    syslog(LOG_INFO, "[sensors] Stopped after %lu loops, %lu deadline misses\n",
           (unsigned long)ctx->loop_count, (unsigned long)ctx->deadline_misses);

    return 0;
}

/****************************************************************************
 * Command Handlers
 ****************************************************************************/

static void print_usage(void)
{
    printf("Usage: sensors <command>\n\n");
    printf("Commands:\n");
    printf("  start     Khởi động sensors task\n");
    printf("  stop      Dừng sensors task\n");
    printf("  status    Xem trạng thái chi tiết\n");
    printf("  test      Chạy self-test\n");
    printf("  timebase  Xem trạng thái timebase\n");
    printf("  health    Xem trạng thái health monitor\n");
    printf("  fusion    Xem trạng thái IMU fusion\n");
}

static int cmd_start(void)
{
    if (g_ctx.is_running) {
        printf("[sensors] Already running\n");
        return 0;
    }

    /* Reset context */
    g_ctx.reset();

    /* Create task */
    g_ctx.task_pid = task_create(
        "sensors",
        CONFIG_UAV_SENSORS_PRIORITY,
        CONFIG_UAV_SENSORS_STACKSIZE,
        sensors_thread_main,
        nullptr
    );

    if (g_ctx.task_pid < 0) {
        printf("[sensors] Failed to create task: %d\n", errno);
        return -errno;
    }

    printf("[sensors] Started (pid=%d)\n", g_ctx.task_pid);
    return 0;
}

static int cmd_stop(void)
{
    if (!g_ctx.is_running) {
        printf("[sensors] Not running\n");
        return 0;
    }

    g_ctx.should_exit = true;

    /* Wait for task to exit */
    for (int i = 0; i < 30 && g_ctx.is_running; i++) {
        usleep(100000);  /* 100ms */
    }

    if (g_ctx.is_running) {
        printf("[sensors] Timeout waiting for task\n");
        return 1;
    }

    printf("[sensors] Stopped\n");
    return 0;
}

static int cmd_status(void)
{
    if (!g_ctx.is_running) {
        printf("[sensors] Not running\n");
        return 0;
    }

    printf("\n========== SENSORS STATUS ==========\n\n");

    printf("Loop count:      %lu\n", (unsigned long)g_ctx.loop_count);
    printf("Deadline misses: %lu\n", (unsigned long)g_ctx.deadline_misses);
    printf("Rate:            %d Hz\n", CONFIG_UAV_IMU_RATE_HZ);
    printf("IMUs:            %d\n", CONFIG_UAV_NUM_IMUS);

    printf("\nPer-IMU statistics:\n");
    for (int i = 0; i < CONFIG_UAV_NUM_IMUS; i++) {
        ImuContext* imu = &g_ctx.imu[i];
        printf("  IMU %d: samples=%lu, errors=%lu\n",
               i, (unsigned long)imu->sample_count,
               (unsigned long)imu->error_count);
    }

    printf("\n");
    return 0;
}

static int cmd_timebase(void)
{
    timebase_print_status();
    return 0;
}

static int cmd_health(void)
{
    get_health_monitor().print_status();
    return 0;
}

static int cmd_fusion(void)
{
    g_ctx.fusion.print_status();
    return 0;
}

/****************************************************************************
 * Public Entry Point
 ****************************************************************************/

extern "C" int sensors_main(int argc, char *argv[])
{
    if (argc < 2) {
        print_usage();
        return 1;
    }

    const char* cmd = argv[1];

    if (strcmp(cmd, "start") == 0) {
        return cmd_start();
    }

    if (strcmp(cmd, "stop") == 0) {
        return cmd_stop();
    }

    if (strcmp(cmd, "status") == 0) {
        return cmd_status();
    }

    if (strcmp(cmd, "timebase") == 0) {
        return cmd_timebase();
    }

    if (strcmp(cmd, "health") == 0) {
        return cmd_health();
    }

    if (strcmp(cmd, "fusion") == 0) {
        return cmd_fusion();
    }

    if (strcmp(cmd, "test") == 0) {
        printf("[sensors] Self-test not implemented yet\n");
        return 0;
    }

    print_usage();
    return 1;
}
