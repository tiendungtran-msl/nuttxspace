/****************************************************************************
 * apps/examples/uav_states_v1/test_icm42688p_quad.cpp
 *
 * Simultaneous data acquisition from 4 ICM-42688-P sensors
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <math.h>

#include "drivers/imu/icm42688p/icm42688p.hpp"
#include "platforms/boards/spi_config.h"

using namespace drivers::imu;

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NUM_SENSORS     4
#define NUM_SAMPLES     20
#define SAMPLE_DELAY_US 50000  // 50ms = 20Hz

#define CALIB_SAMPLES   200
#define CALIB_DELAY_US  5000   // 5ms = 200Hz

static constexpr float GRAVITY_M_S2 = 9.80665f;

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const char *sensor_names[NUM_SENSORS] = {
    "IMU0", "IMU1", "IMU2", "IMU3"
};

static const uint32_t sensor_devids[NUM_SENSORS] = {
    SPIDEV_IMU, SPIDEV_IMU1, SPIDEV_IMU2, SPIDEV_IMU3
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void print_sensor_data(int sensor_id, int sample_num, 
                              const ICM42688P::Data &data)
{
    const float nA = accel_norm(data);
    printf("[%s] %3d: A[%7.3f %7.3f %7.3f] |A|=%5.2f  G[%7.3f %7.3f %7.3f] T=%5.1f°C\n",
           sensor_names[sensor_id],
           sample_num,
           data.accel[0], data.accel[1], data.accel[2],
           (double)nA,
           data.gyro[0], data.gyro[1], data.gyro[2],
           data.temperature);
}

static float accel_norm(const ICM42688P::Data &d)
{
    return sqrtf(d.accel[0] * d.accel[0] + d.accel[1] * d.accel[1] + d.accel[2] * d.accel[2]);
}

static void calibrate_stationary(ICM42688P *sensors[NUM_SENSORS], const bool sensor_ok[NUM_SENSORS])
{
    printf("\n");
    printf("=== Stationary calibration (level placement) ===\n");
    printf("Keep the rig completely LEVEL and STILL for ~%u ms.\n", (unsigned)((CALIB_SAMPLES * CALIB_DELAY_US) / 1000));
    printf("Assuming +Z axis points UP (gravity = [0, 0, +g]).\n");
    printf("Will estimate: gyro bias, accel scale, and accel bias per axis.\n\n");

    for (int i = 0; i < NUM_SENSORS; i++) {
        if (!sensor_ok[i]) {
            continue;
        }

        float sum_g[3] = {0.0f, 0.0f, 0.0f};
        float sum_a[3] = {0.0f, 0.0f, 0.0f};
        int ok = 0;

        ICM42688P::Data d;
        for (int n = 0; n < CALIB_SAMPLES; n++) {
            const int ret = sensors[i]->read(d);
            if (ret == 0) {
                for (int k = 0; k < 3; k++) {
                    sum_g[k] += d.gyro[k];
                    sum_a[k] += d.accel[k];
                }
                ok++;
            }
            usleep(CALIB_DELAY_US);
        }

        if (ok < (CALIB_SAMPLES * 3) / 4) {
            printf("[%s] Calibration skipped: too many read failures (%d/%d).\n", sensor_names[i], ok, CALIB_SAMPLES);
            continue;
        }

        float mean_g[3];
        float mean_a[3];
        for (int k = 0; k < 3; k++) {
            mean_g[k] = sum_g[k] / ok;
            mean_a[k] = sum_a[k] / ok;
        }

        // Print raw mean for debugging
        printf("[%s] RAW mean_a=[%7.3f %7.3f %7.3f] mean_g=[%7.4f %7.4f %7.4f]\n",
               sensor_names[i],
               (double)mean_a[0], (double)mean_a[1], (double)mean_a[2],
               (double)mean_g[0], (double)mean_g[1], (double)mean_g[2]);

        // Step 1: Compute isotropic scale to normalize |A| -> g
        const float norm_a = sqrtf(mean_a[0] * mean_a[0] + mean_a[1] * mean_a[1] + mean_a[2] * mean_a[2]);
        float scale_corr = 1.0f;
        if (norm_a > 1e-3f) {
            scale_corr = GRAVITY_M_S2 / norm_a;
        }

        // Step 2: After scaling, expected accel when level is [0, 0, +g] (or [0, 0, -g] if Z-down)
        // Compute accel bias = scaled_mean - expected
        float mean_a_scaled[3] = {
            mean_a[0] * scale_corr,
            mean_a[1] * scale_corr,
            mean_a[2] * scale_corr
        };

        // Determine if Z points up or down based on sign of mean_a[2]
        float expected_z = (mean_a[2] >= 0.0f) ? GRAVITY_M_S2 : -GRAVITY_M_S2;
        float accel_bias[3] = {
            mean_a_scaled[0] - 0.0f,       // X should be 0 when level
            mean_a_scaled[1] - 0.0f,       // Y should be 0 when level  
            mean_a_scaled[2] - expected_z  // Z should be ±g when level
        };

        // Apply calibration: gyro bias, accel scale, accel bias
        sensors[i]->set_gyro_bias(mean_g);
        sensors[i]->set_accel_scale_correction(scale_corr);
        sensors[i]->set_accel_bias(accel_bias);

        printf("[%s] CALIB: gyro_bias=[%+.4f %+.4f %+.4f] scale=%.4f bias=[%+.3f %+.3f %+.3f]\n",
               sensor_names[i],
               (double)mean_g[0], (double)mean_g[1], (double)mean_g[2],
               (double)scale_corr,
               (double)accel_bias[0], (double)accel_bias[1], (double)accel_bias[2]);
    }

    printf("\nAfter calibration, all IMUs should show A ~ [0, 0, ±9.81] when level.\n\n");
}

struct CfgReg {
    icm42688p_bank_t bank;
    uint8_t reg;
    const char *name;
};

static const CfgReg cfg_regs[] = {
    // Bank 0 (main config)
    {BANK_0, ICM42688P_INTF_CONFIG0,       "INTF_CONFIG0"},
    {BANK_0, ICM42688P_INTF_CONFIG1,       "INTF_CONFIG1"},
    {BANK_0, ICM42688P_PWR_MGMT0,          "PWR_MGMT0"},
    {BANK_0, ICM42688P_GYRO_CONFIG0,       "GYRO_CONFIG0"},
    {BANK_0, ICM42688P_ACCEL_CONFIG0,      "ACCEL_CONFIG0"},
    {BANK_0, ICM42688P_GYRO_CONFIG1,       "GYRO_CONFIG1"},
    {BANK_0, ICM42688P_ACCEL_CONFIG1,      "ACCEL_CONFIG1"},
    {BANK_0, ICM42688P_GYRO_ACCEL_CONFIG0, "GYRO_ACCEL_CONFIG0"},
    {BANK_0, ICM42688P_INT_CONFIG,         "INT_CONFIG"},
    {BANK_0, ICM42688P_INT_CONFIG1,        "INT_CONFIG1"},
    {BANK_0, ICM42688P_INT_SOURCE0,        "INT_SOURCE0"},
    {BANK_0, ICM42688P_FIFO_CONFIG,        "FIFO_CONFIG"},

    // Bank 1/2 (AAF static config written in configure())
    {BANK_1, 0x0B,                         "B1:GYRO_CFG_STATIC3"},
    {BANK_1, 0x0C,                         "B1:GYRO_CFG_STATIC4"},
    {BANK_1, 0x0D,                         "B1:GYRO_CFG_STATIC5"},
    {BANK_2, 0x03,                         "B2:ACC_CFG_STATIC2"},
    {BANK_2, 0x04,                         "B2:ACC_CFG_STATIC3"},
    {BANK_2, 0x05,                         "B2:ACC_CFG_STATIC4"},
};

static int dump_and_compare_config(ICM42688P *sensors[NUM_SENSORS], const bool sensor_ok[NUM_SENSORS])
{
    uint8_t baseline[sizeof(cfg_regs) / sizeof(cfg_regs[0])] = {0};
    bool baseline_valid = false;
    int baseline_idx = -1;

    // Pick first active IMU as baseline
    for (int i = 0; i < NUM_SENSORS; i++) {
        if (sensor_ok[i]) {
            baseline_idx = i;
            break;
        }
    }

    if (baseline_idx < 0) {
        return -1;
    }

    printf("2. Verifying configuration consistency (read-back registers):\n");

    // Read baseline
    for (unsigned r = 0; r < (sizeof(cfg_regs) / sizeof(cfg_regs[0])); r++) {
        uint8_t v = 0;
        const int ret = sensors[baseline_idx]->debug_read_reg(cfg_regs[r].bank, cfg_regs[r].reg, v);
        if (ret != 0) {
            printf("   [%s] read %s FAILED (err %d)\n", sensor_names[baseline_idx], cfg_regs[r].name, ret);
            return ret;
        }
        baseline[r] = v;
    }
    baseline_valid = true;

    int mismatch_total = 0;

    for (int i = 0; i < NUM_SENSORS; i++) {
        if (!sensor_ok[i]) continue;

        int mismatch_count = 0;
        printf("   [%s] ", sensor_names[i]);

        for (unsigned r = 0; r < (sizeof(cfg_regs) / sizeof(cfg_regs[0])); r++) {
            uint8_t v = 0;
            const int ret = sensors[i]->debug_read_reg(cfg_regs[r].bank, cfg_regs[r].reg, v);
            if (ret != 0) {
                printf("%s=ERR(%d) ", cfg_regs[r].name, ret);
                mismatch_count++;
                continue;
            }

            const bool mismatch = baseline_valid && (v != baseline[r]);
            if (mismatch) {
                mismatch_count++;
            }

            // Mark mismatches with '!'
            printf("%s=0x%02X%s ", cfg_regs[r].name, v, mismatch ? "!" : "");
        }

        if (i == baseline_idx) {
            printf("(baseline)\n");
        } else if (mismatch_count == 0) {
            printf("=> MATCH\n");
        } else {
            printf("=> MISMATCH(%d)\n", mismatch_count);
        }

        mismatch_total += (i == baseline_idx) ? 0 : mismatch_count;
    }

    if (mismatch_total == 0) {
        printf("   Result: All active IMUs have identical config registers.\n\n");
        return 0;
    }

    printf("   Result: MISMATCH detected. '!' marks registers differing from %s.\n\n", sensor_names[baseline_idx]);
    return 1;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

extern "C" int main(int argc, char *argv[])
{
    ICM42688P *sensors[NUM_SENSORS];
    bool sensor_ok[NUM_SENSORS];
    int active_sensors = 0;
    int ret;

    const bool do_calib = (argc >= 2) &&
                          (!strcmp(argv[1], "calib") || !strcmp(argv[1], "--calib") || !strcmp(argv[1], "-c"));

    printf("\n");
    printf("========================================\n");
    printf("  ICM-42688-P Quad-Sensor Test\n");
    printf("========================================\n\n");

    // Step 1: Create and initialize all sensors
    printf("1. Initializing sensors...\n");
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        printf("   [%s] ", sensor_names[i]);
        
        sensors[i] = new ICM42688P(NUTTX_SPI_BUS_IMU, sensor_devids[i]);
        
        ret = sensors[i]->initialize();
        
        if (ret == 0)
        {
            printf("✓ OK\n");
            sensor_ok[i] = true;
            active_sensors++;
        }
        else
        {
            printf("✗ FAILED (error %d)\n", ret);
            sensor_ok[i] = false;
        }
    }

    printf("\n   Active sensors: %d/%d\n\n", active_sensors, NUM_SENSORS);

    if (active_sensors == 0)
    {
        printf("❌ No sensors detected!\n");
        printf("\nCheck:\n");
        printf("  - SPI wiring (MISO, MOSI, SCK)\n");
        printf("  - CS pins for each sensor\n");
        printf("  - Power 3.3V\n");
        printf("  - Ground connected\n\n");
        
        for (int i = 0; i < NUM_SENSORS; i++)
        {
            delete sensors[i];
        }
        return -1;
    }

    // Step 2: Read-back and compare config
    (void)dump_and_compare_config(sensors, sensor_ok);

    // Optional: stationary calibration
    if (do_calib) {
        calibrate_stationary(sensors, sensor_ok);
    } else {
        printf("Tip: run 'test_icm_quad calib' to estimate gyro bias + accel scale while stationary.\n\n");
    }

    // Step 3: Acquire data from all active sensors
    printf("3. Acquiring %d samples @ 20Hz:\n", NUM_SAMPLES);
    printf("-----------------------------------------------------------\n");

    ICM42688P::Data data;
    int success_count[NUM_SENSORS] = {0};

    for (int sample = 0; sample < NUM_SAMPLES; sample++)
    {
        for (int i = 0; i < NUM_SENSORS; i++)
        {
            if (!sensor_ok[i]) continue;

            ret = sensors[i]->read(data);
            
            if (ret == 0)
            {
                print_sensor_data(i, sample, data);
                success_count[i]++;
            }
            else
            {
                printf("[%s] %3d: READ FAILED (error %d)\n",
                       sensor_names[i], sample, ret);
            }
        }
        
        if (sample < NUM_SAMPLES - 1)
        {
            printf("\n");
            usleep(SAMPLE_DELAY_US);
        }
    }

    printf("-----------------------------------------------------------\n\n");

    // Step 4: Print statistics
    printf("4. Statistics:\n");
    printf("   Sensor  Active  Success  Rate\n");
    printf("   ------  ------  -------  ----\n");
    
    int total_success = 0;
    int total_expected = 0;
    
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        if (sensor_ok[i])
        {
            float rate = (100.0f * success_count[i]) / NUM_SAMPLES;
            printf("   %-6s    ✓     %3d/%2d   %5.1f%%\n",
                   sensor_names[i], success_count[i], NUM_SAMPLES, rate);
            total_success += success_count[i];
            total_expected += NUM_SAMPLES;
        }
        else
        {
            printf("   %-6s    ✗       -/-       -\n", sensor_names[i]);
        }
    }
    
    printf("   ------  ------  -------  ----\n");
    
    if (total_expected > 0)
    {
        float total_rate = (100.0f * total_success) / total_expected;
        printf("   Total          %3d/%2d   %5.1f%%\n",
               total_success, total_expected, total_rate);
    }

    printf("\n");

    // Step 5: Cleanup
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        delete sensors[i];
    }

    // Final result
    if (total_success >= total_expected * 0.95)  // 95% success rate
    {
        printf("✓✓✓ TEST PASSED ✓✓✓\n");
        printf("All active sensors working properly!\n");
    }
    else if (total_success >= total_expected * 0.75)  // 75% success rate
    {
        printf("⚠ TEST PARTIAL ⚠\n");
        printf("Some reads failed, check connections\n");
    }
    else
    {
        printf("❌ TEST FAILED ❌\n");
        printf("Too many read failures\n");
    }
    
    printf("\n");

    return (total_success >= total_expected * 0.95) ? 0 : -1;
}
