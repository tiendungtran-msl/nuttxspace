/****************************************************************************
 * apps/uav/lib/platform/timebase.c
 *
 * MASTER TIMEBASE - Implementation
 *
 * MỤC ĐÍCH:
 * - Cung cấp timer-based tick generation
 * - Phân phối ticks cho các domains
 * - Đo lường jitter để monitor timing quality
 *
 * IMPLEMENTATION NOTES:
 * - Sử dụng NuttX interval timer (không cần hardware timer trực tiếp)
 * - Timer signal handler chỉ post semaphores (minimal processing)
 * - Thread-safe thông qua atomic operations và semaphores
 *
 * TRÊN STM32H7:
 * - Có thể upgrade lên hardware timer (TIM5) để giảm jitter hơn nữa
 * - Hiện tại dùng software timer qua POSIX API cho portability
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "timebase.h"
#include "hrt.h"

#include <nuttx/config.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <signal.h>
#include <time.h>
#include <syslog.h>
#include <nuttx/clock.h>
#include <nuttx/semaphore.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Global timebase state - singleton */
static timebase_state_t g_timebase;

/* Timer ID cho interval timer */
static timer_t g_timer_id;

/* Timestamp của tick trước để tính jitter */
static uint64_t g_last_tick_time = 0;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/**
 * @brief Timer signal handler - Master Tick ISR
 *
 * CRITICAL: Handler này được gọi từ signal context.
 * Chỉ được làm các việc async-signal-safe:
 * - Set volatile flags
 * - Post semaphores
 * - KHÔNG malloc, printf, mutex lock, etc.
 *
 * WCET Budget: < 5 µs
 */
static void timer_signal_handler(int signo, siginfo_t *info, void *context)
{
    (void)signo;
    (void)info;
    (void)context;

    uint64_t now = hrt_absolute_time();
    uint32_t phase;

    /*-----------------------------------------------------------------------
     * Update timestamps (atomic on 32-bit ARM with proper alignment)
     *-----------------------------------------------------------------------*/

    g_timebase.tick_count++;
    g_timebase.timestamp_us = now;

    /*-----------------------------------------------------------------------
     * Calculate phase within superframe
     * Superframe = 80 ticks = 10 ms (với 8 kHz master)
     *-----------------------------------------------------------------------*/

    phase = g_timebase.tick_count % TIMEBASE_SUPERFRAME_TICKS;
    g_timebase.tick_phase = phase;

    /*-----------------------------------------------------------------------
     * Determine which domains to trigger
     *-----------------------------------------------------------------------*/

    /* Always trigger FAST domain (every tick) */
    g_timebase.tick_flags = TICK_FLAG_FAST;
    nxsem_post(&g_timebase.domain_sem[DOMAIN_FAST]);

    /* Trigger MEDIUM domain every 8 ticks (1 kHz) */
    if ((phase % CONFIG_UAV_MEDIUM_DIVIDER) == 0) {
        g_timebase.tick_flags |= TICK_FLAG_MEDIUM;
        nxsem_post(&g_timebase.domain_sem[DOMAIN_MEDIUM]);
    }

    /* Trigger SLOW domain at phase 0 (100 Hz) */
    if (phase == 0) {
        g_timebase.tick_flags |= TICK_FLAG_SLOW;
        nxsem_post(&g_timebase.domain_sem[DOMAIN_SLOW]);
    }

    /*-----------------------------------------------------------------------
     * Jitter measurement
     *-----------------------------------------------------------------------*/

    if (g_last_tick_time > 0) {
        uint64_t delta_us = now - g_last_tick_time;
        uint32_t delta_ns = (uint32_t)(delta_us * 1000);

        /* Update min/max */
        if (g_timebase.jitter.samples == 0 ||
            delta_ns < g_timebase.jitter.min_period_ns) {
            g_timebase.jitter.min_period_ns = delta_ns;
        }
        if (delta_ns > g_timebase.jitter.max_period_ns) {
            g_timebase.jitter.max_period_ns = delta_ns;
        }

        /* Accumulate for average (simple running sum) */
        g_timebase.jitter.avg_period_ns += delta_ns;
        g_timebase.jitter.samples++;

        /* Deadline miss detection (> 1.5x expected period) */
        uint32_t expected_ns = TIMEBASE_MASTER_PERIOD_US * 1000;
        if (delta_ns > (expected_ns * 3 / 2)) {
            g_timebase.jitter.deadline_misses++;
        }
    }

    g_last_tick_time = now;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/**
 * @brief Khởi tạo Master Timebase
 */
int timebase_init(void)
{
    struct sigevent sev;
    struct itimerspec its;
    struct sigaction sa;
    int ret;

    if (g_timebase.initialized) {
        syslog(LOG_WARNING, "[timebase] Already initialized\n");
        return 0;
    }

    /*-----------------------------------------------------------------------
     * Initialize state
     *-----------------------------------------------------------------------*/

    memset(&g_timebase, 0, sizeof(g_timebase));

    /* Initialize semaphores for each domain */
    for (int i = 0; i < DOMAIN_COUNT; i++) {
        ret = nxsem_init(&g_timebase.domain_sem[i], 0, 0);
        if (ret < 0) {
            syslog(LOG_ERR, "[timebase] Failed to init semaphore %d: %d\n", i, ret);
            return ret;
        }

        /* Set to not wait forever - allow wake on post */
        nxsem_set_protocol(&g_timebase.domain_sem[i], SEM_PRIO_NONE);
    }

    /*-----------------------------------------------------------------------
     * Setup signal handler for timer
     *-----------------------------------------------------------------------*/

    memset(&sa, 0, sizeof(sa));
    sa.sa_sigaction = timer_signal_handler;
    sa.sa_flags = SA_SIGINFO;
    sigemptyset(&sa.sa_mask);

    ret = sigaction(SIGRTMIN, &sa, NULL);
    if (ret < 0) {
        syslog(LOG_ERR, "[timebase] Failed to setup signal handler: %d\n", errno);
        return -errno;
    }

    /*-----------------------------------------------------------------------
     * Create interval timer
     *-----------------------------------------------------------------------*/

    memset(&sev, 0, sizeof(sev));
    sev.sigev_notify = SIGEV_SIGNAL;
    sev.sigev_signo = SIGRTMIN;
    sev.sigev_value.sival_ptr = &g_timer_id;

    ret = timer_create(CLOCK_REALTIME, &sev, &g_timer_id);
    if (ret < 0) {
        syslog(LOG_ERR, "[timebase] Failed to create timer: %d\n", errno);
        return -errno;
    }

    /*-----------------------------------------------------------------------
     * Start timer với period = TIMEBASE_MASTER_PERIOD_US
     *-----------------------------------------------------------------------*/

    memset(&its, 0, sizeof(its));
    its.it_value.tv_sec = 0;
    its.it_value.tv_nsec = TIMEBASE_MASTER_PERIOD_US * 1000;  /* µs → ns */
    its.it_interval.tv_sec = 0;
    its.it_interval.tv_nsec = TIMEBASE_MASTER_PERIOD_US * 1000;

    ret = timer_settime(g_timer_id, 0, &its, NULL);
    if (ret < 0) {
        syslog(LOG_ERR, "[timebase] Failed to start timer: %d\n", errno);
        timer_delete(g_timer_id);
        return -errno;
    }

    /*-----------------------------------------------------------------------
     * Mark as initialized and running
     *-----------------------------------------------------------------------*/

    g_timebase.initialized = true;
    g_timebase.running = true;
    g_last_tick_time = hrt_absolute_time();

    syslog(LOG_INFO, "[timebase] Initialized @ %d Hz (period=%d µs)\n",
           CONFIG_UAV_MASTER_TICK_HZ, TIMEBASE_MASTER_PERIOD_US);
    syslog(LOG_INFO, "[timebase] FAST=%d Hz, MEDIUM=%d Hz, SLOW=%d Hz\n",
           CONFIG_UAV_MASTER_TICK_HZ,
           CONFIG_UAV_MASTER_TICK_HZ / CONFIG_UAV_MEDIUM_DIVIDER,
           CONFIG_UAV_MASTER_TICK_HZ / CONFIG_UAV_SLOW_DIVIDER);

    return 0;
}

/**
 * @brief Dừng và cleanup timebase
 */
void timebase_deinit(void)
{
    if (!g_timebase.initialized) {
        return;
    }

    /* Stop timer */
    struct itimerspec its;
    memset(&its, 0, sizeof(its));
    timer_settime(g_timer_id, 0, &its, NULL);

    /* Delete timer */
    timer_delete(g_timer_id);

    /* Post all semaphores to unblock waiting tasks */
    for (int i = 0; i < DOMAIN_COUNT; i++) {
        nxsem_post(&g_timebase.domain_sem[i]);
        nxsem_destroy(&g_timebase.domain_sem[i]);
    }

    g_timebase.running = false;
    g_timebase.initialized = false;

    syslog(LOG_INFO, "[timebase] Deinitialized after %llu ticks, %lu deadline misses\n",
           (unsigned long long)g_timebase.tick_count,
           (unsigned long)g_timebase.jitter.deadline_misses);
}

/**
 * @brief Kiểm tra timebase đã được khởi tạo chưa
 */
bool timebase_is_initialized(void)
{
    return g_timebase.initialized;
}

/**
 * @brief Lấy timestamp hiện tại (microseconds)
 */
uint64_t timebase_get_timestamp(void)
{
    return hrt_absolute_time();
}

/**
 * @brief Lấy tick count hiện tại
 */
uint64_t timebase_get_tick_count(void)
{
    return g_timebase.tick_count;
}

/**
 * @brief Lấy phase hiện tại trong superframe
 */
uint32_t timebase_get_phase(void)
{
    return g_timebase.tick_phase;
}

/**
 * @brief Đợi tick của một domain cụ thể
 */
int timebase_wait_domain(timebase_domain_t domain)
{
    if (domain >= DOMAIN_COUNT) {
        return -EINVAL;
    }

    if (!g_timebase.running) {
        return -ENODEV;
    }

    int ret = nxsem_wait(&g_timebase.domain_sem[domain]);
    if (ret < 0) {
        return ret;
    }

    return 0;
}

/**
 * @brief Đợi tick với timeout
 */
int timebase_wait_domain_timeout(timebase_domain_t domain, uint32_t timeout_us)
{
    if (domain >= DOMAIN_COUNT) {
        return -EINVAL;
    }

    if (!g_timebase.running) {
        return -ENODEV;
    }

    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);

    /* Add timeout */
    ts.tv_nsec += (timeout_us % 1000000) * 1000;
    ts.tv_sec += timeout_us / 1000000;
    if (ts.tv_nsec >= 1000000000) {
        ts.tv_nsec -= 1000000000;
        ts.tv_sec++;
    }

    int ret = nxsem_timedwait(&g_timebase.domain_sem[domain], &ts);
    if (ret < 0) {
        if (ret == -ETIMEDOUT) {
            return -ETIMEDOUT;
        }
        return -EINTR;
    }

    return 0;
}

/**
 * @brief Signal async domain từ ISR hoặc external event
 */
void timebase_signal_async(void)
{
    if (g_timebase.running) {
        g_timebase.tick_flags |= TICK_FLAG_ASYNC;
        nxsem_post(&g_timebase.domain_sem[DOMAIN_ASYNC]);
    }
}

/**
 * @brief Lấy thống kê jitter
 */
void timebase_get_jitter_stats(timebase_jitter_stats_t *stats)
{
    if (stats != NULL) {
        *stats = g_timebase.jitter;

        /* Calculate average */
        if (stats->samples > 0) {
            stats->avg_period_ns = stats->avg_period_ns / stats->samples;
        }
    }
}

/**
 * @brief Reset thống kê jitter
 */
void timebase_reset_jitter_stats(void)
{
    memset(&g_timebase.jitter, 0, sizeof(g_timebase.jitter));
    g_last_tick_time = hrt_absolute_time();
}

/**
 * @brief Lấy tần số thực tế của mỗi domain
 */
uint32_t timebase_get_domain_hz(timebase_domain_t domain)
{
    switch (domain) {
        case DOMAIN_FAST:
            return CONFIG_UAV_MASTER_TICK_HZ;
        case DOMAIN_MEDIUM:
            return CONFIG_UAV_MASTER_TICK_HZ / CONFIG_UAV_MEDIUM_DIVIDER;
        case DOMAIN_SLOW:
            return CONFIG_UAV_MASTER_TICK_HZ / CONFIG_UAV_SLOW_DIVIDER;
        case DOMAIN_ASYNC:
            return 0;  /* Event-driven, no fixed rate */
        default:
            return 0;
    }
}

/**
 * @brief In trạng thái debug của timebase
 */
void timebase_print_status(void)
{
    timebase_jitter_stats_t stats;
    timebase_get_jitter_stats(&stats);

    printf("[timebase] Status:\n");
    printf("  Initialized:    %s\n", g_timebase.initialized ? "yes" : "no");
    printf("  Running:        %s\n", g_timebase.running ? "yes" : "no");
    printf("  Tick count:     %llu\n", (unsigned long long)g_timebase.tick_count);
    printf("  Phase:          %lu / %d\n",
           (unsigned long)g_timebase.tick_phase, TIMEBASE_SUPERFRAME_TICKS);
    printf("  Master rate:    %d Hz\n", CONFIG_UAV_MASTER_TICK_HZ);
    printf("\n  Jitter stats:\n");
    printf("    Samples:      %lu\n", (unsigned long)stats.samples);
    printf("    Min period:   %lu ns\n", (unsigned long)stats.min_period_ns);
    printf("    Max period:   %lu ns\n", (unsigned long)stats.max_period_ns);
    printf("    Avg period:   %llu ns\n", (unsigned long long)stats.avg_period_ns);
    printf("    Expected:     %d µs (%d ns)\n",
           TIMEBASE_MASTER_PERIOD_US, TIMEBASE_MASTER_PERIOD_US * 1000);
    printf("    Deadline miss: %lu\n", (unsigned long)stats.deadline_misses);
}
