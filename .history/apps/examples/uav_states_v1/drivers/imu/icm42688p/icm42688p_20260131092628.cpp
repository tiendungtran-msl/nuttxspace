/****************************************************************************
 * apps/examples/uav_states_v1/drivers/imu/icm42688p/icm42688p.cpp
 ****************************************************************************/

#include "icm42688p.hpp"
#include "../../../platforms/nuttx/hrt/hrt.h"
#include "../../../platforms/boards/spi_config.h"
#include "../../../lib/utils/debug.hpp"

#include <unistd.h>
#include <math.h>
#include <string.h>

namespace drivers {
    namespace imu {

        // Constants from PX4
        static constexpr uint8_t DIR_READ = 0x80;
        static constexpr float TEMPERATURE_SENSITIVITY = 132.48f;
        static constexpr float TEMPERATURE_OFFSET = 25.0f;

        ICM42688P::ICM42688P(uint8_t bus, uint32_t cs)
            : spi::Device(
                spi::Config{bus, cs, SPIDEV_MODE3, 1000000, spi::LockMode::THREADS},
                DeviceType::ICM42688P)
            , _gyro_scale(1.0f)
            , _accel_scale(1.0f)
            , _current_bank(BANK_0)
            , _initialized(false)
            , _read_count(0)
            , _error_count(0)
            , _last_read_time(0)
        {
            // Calibration objects initialized to identity by default
        }

        // Legacy API implementation using calibration objects
        void ICM42688P::set_gyro_bias(const float bias_rad_s[3])
        {
            calibration::Vector3f bias(bias_rad_s[0], bias_rad_s[1], bias_rad_s[2]);
            _gyro_cal.set_offset(bias);
        }

        void ICM42688P::set_accel_bias(const float bias_m_s2[3])
        {
            calibration::Vector3f bias(bias_m_s2[0], bias_m_s2[1], bias_m_s2[2]);
            _accel_cal.set_offset(bias);
        }

        void ICM42688P::set_accel_scale_correction(float scale)
        {
            // Apply isotropic scale correction to all axes
            calibration::Vector3f scale_vec(scale, scale, scale);
            _accel_cal.set_scale(scale_vec);
        }

        void ICM42688P::get_gyro_bias(float bias_rad_s[3]) const
        {
            const calibration::Vector3f& bias = _gyro_cal.get_offset();
            bias_rad_s[0] = bias.x;
            bias_rad_s[1] = bias.y;
            bias_rad_s[2] = bias.z;
        }

        void ICM42688P::get_accel_bias(float bias_m_s2[3]) const
        {
            const calibration::Vector3f& bias = _accel_cal.get_offset();
            bias_m_s2[0] = bias.x;
            bias_m_s2[1] = bias.y;
            bias_m_s2[2] = bias.z;
        }

        float ICM42688P::get_accel_scale_correction() const
        {
            // Return average scale (assuming isotropic correction)
            const calibration::Vector3f& scale = _accel_cal.get_scale();
            return (scale.x + scale.y + scale.z) / 3.0f;
        }

        ICM42688P::~ICM42688P()
        {
        }

        int ICM42688P::probe()
        {
            int ret = select_bank(BANK_0);
            if (ret != 0) {
                DRIVER_ERR("Failed to select bank 0");
                return ret;
            }
            
            uint8_t whoami;
            ret = read_reg(ICM42688P_WHO_AM_I, whoami);
            if (ret != 0) {
                DRIVER_ERR("Failed to read WHO_AM_I: %d", ret);
                return ret;
            }
            
            if (whoami != ICM42688P_WHO_AM_I_VALUE) {
                DRIVER_ERR("Wrong WHO_AM_I: 0x%02X (expected 0x%02X)", 
                        whoami, ICM42688P_WHO_AM_I_VALUE);
                return -ENODEV;
            }
            
            DRIVER_INFO("✓ ICM-42688-P detected, WHO_AM_I=0x%02X", whoami);
            return 0;
        }

        int ICM42688P::initialize()
        {
            DRIVER_INFO("Initializing ICM-42688-P...");
            
            // Reset calibration to identity (avoid stale/uninitialized values)
            _gyro_cal.reset();
            _accel_cal.reset();
            
            int ret = init();
            if (ret != 0) {
                DRIVER_ERR("Base init failed: %d", ret);
                return ret;
            }
            
            ret = reset();
            if (ret != 0) {
                DRIVER_ERR("Reset failed: %d", ret);
                return ret;
            }
            
            ret = configure();
            if (ret != 0) {
                DRIVER_ERR("Configuration failed: %d", ret);
                return ret;
            }
            
            // Switch to high-speed SPI (24 MHz max per datasheet, use 10 MHz safe)
            set_frequency(10000000);
            
            _initialized = true;
            DRIVER_INFO("✓ ICM-42688-P initialized @ %u Hz", get_frequency());
            
            return 0;
        }

        int ICM42688P::reset()
        {
            DRIVER_INFO("Resetting sensor...");
            
            int ret = select_bank(BANK_0);
            if (ret != 0) return ret;
            
            // Soft reset (from PX4: DEVICE_CONFIG, bit 0)
            ret = write_reg(ICM42688P_DEVICE_CONFIG, 0x01);
            if (ret != 0) {
                DRIVER_ERR("Soft reset write failed");
                return ret;
            }
            
            // Wait for reset to complete (PX4 uses 1ms)
            usleep(1000);
            
            // Wait for device to be ready
            const int max_tries = 10;
            for (int i = 0; i < max_tries; i++) {
                uint8_t reset_done;
                ret = read_reg(ICM42688P_INT_STATUS, reset_done);
                
                if (ret == 0 && (reset_done & (1 << 4))) {  // RESET_DONE_INT bit
                    DRIVER_INFO("✓ Reset complete after %d ms", i + 1);
                    _current_bank = BANK_0;
                    return 0;
                }
                
                usleep(1000);
            }
            
            DRIVER_WARN("Reset status not confirmed, continuing anyway");
            _current_bank = BANK_0;
            return 0;
        }

        int ICM42688P::configure()
        {
            int ret;
            
            // === BANK 0 Configuration ===
            ret = select_bank(BANK_0);
            if (ret != 0) return ret;
            
            // 1. Configure interface (from PX4)
            // - Big endian sensor data
            // - Disable I2C
            uint8_t intf_config0 = (1 << 4) | 0x03;  // SENSOR_DATA_ENDIAN | UI_SIFS_CFG_DISABLE_I2C
            ret = register_write_verified(BANK_0, ICM42688P_INTF_CONFIG0, intf_config0);
            if (ret != 0) {
                DRIVER_ERR("INTF_CONFIG0 failed");
                return ret;
            }
            
            // 2. Disable AFSR (Adaptive Full Scale Range) - from PX4
            ret = register_write(BANK_0, ICM42688P_INTF_CONFIG1, 0x80);  // AFSR_CLEAR
            if (ret != 0) return ret;
            
            // 3. Power management: Gyro and Accel in Low Noise mode
            uint8_t pwr_mgmt0 = PWR_MGMT0_GYRO_MODE_LN | PWR_MGMT0_ACCEL_MODE_LN;
            ret = register_write_verified(BANK_0, ICM42688P_PWR_MGMT0, pwr_mgmt0);
            if (ret != 0) {
                DRIVER_ERR("PWR_MGMT0 failed");
                return ret;
            }
            
            // Wait for sensors to power up
            usleep(30000);  // 30ms from PX4
            
            // 4. Set ranges
            ret = set_gyro_range(GyroRange::DPS_2000);
            if (ret != 0) return ret;
            
            ret = set_accel_range(AccelRange::G16);
            if (ret != 0) return ret;
            
            // 5. Set sample rate to 8kHz (matching PX4)
            ret = set_sample_rate(ODR::HZ_8000);
            if (ret != 0) return ret;
            
            // 6. Configure filters
            // Set UI filter order to 1st order (from PX4)
            ret = register_write(BANK_0, ICM42688P_GYRO_CONFIG1, 0x0C);  // GYRO_UI_FILT_ORD = 00
            if (ret != 0) return ret;
            
            ret = register_write(BANK_0, ICM42688P_ACCEL_CONFIG1, 0x18);  // ACCEL_UI_FILT_ORD = 00
            if (ret != 0) return ret;
            
            // Set UI filter bandwidth to ODR/2
            ret = register_write(BANK_0, ICM42688P_GYRO_ACCEL_CONFIG0, 0x00);
            if (ret != 0) return ret;
            
            // === BANK 1 Configuration (Anti-Aliasing Filter) ===
            ret = select_bank(BANK_1);
            if (ret != 0) return ret;
            
            // Configure 585Hz AAF for gyro (from PX4)
            ret = register_write(BANK_1, 0x0B, 0x0D);  // GYRO_CONFIG_STATIC3: GYRO_AAF_DELT = 13
            if (ret != 0) return ret;
            
            ret = register_write(BANK_1, 0x0C, 0xAA);  // GYRO_CONFIG_STATIC4: GYRO_AAF_DELTSQR_LSB = 170
            if (ret != 0) return ret;
            
            ret = register_write(BANK_1, 0x0D, 0x80);  // GYRO_CONFIG_STATIC5
            if (ret != 0) return ret;
            
            // === BANK 2 Configuration (Accel AAF) ===
            ret = select_bank(BANK_2);
            if (ret != 0) return ret;
            
            ret = register_write(BANK_2, 0x03, 0x1A);  // ACCEL_CONFIG_STATIC2
            if (ret != 0) return ret;
            
            ret = register_write(BANK_2, 0x04, 0xAA);  // ACCEL_CONFIG_STATIC3
            if (ret != 0) return ret;
            
            ret = register_write(BANK_2, 0x05, 0x80);  // ACCEL_CONFIG_STATIC4
            if (ret != 0) return ret;
            
            // === Back to BANK 0 ===
            ret = select_bank(BANK_0);
            if (ret != 0) return ret;
            
            // 7. Configure interrupt
            ret = configure_interrupt();
            if (ret != 0) {
                DRIVER_WARN("Interrupt config failed (non-critical)");
            }
            
            DRIVER_INFO("✓ Configuration complete");
            return 0;
        }

        int ICM42688P::debug_read_reg(icm42688p_bank_t bank, uint8_t reg, uint8_t &value)
        {
            const icm42688p_bank_t previous_bank = _current_bank;

            int ret = select_bank(bank);
            if (ret != 0) {
                return ret;
            }

            ret = read_reg(reg, value);

            /* Best-effort restore bank */
            (void)select_bank(previous_bank);
            return ret;
        }

        int ICM42688P::configure_interrupt()
        {
            // INT1 configuration (from PX4)
            // - Push-pull
            // - Active high  
            // - Pulsed mode
            uint8_t int_config = 0x00;  // All cleared = push-pull, active high, pulsed
            int ret = register_write(BANK_0, ICM42688P_INT_CONFIG, int_config);
            if (ret != 0) return ret;
            
            // Clear INT_ASYNC_RESET (from PX4 comment)
            ret = register_write(BANK_0, ICM42688P_INT_CONFIG1, 0x00);
            if (ret != 0) return ret;
            
            // Enable data ready interrupt
            ret = register_write(BANK_0, ICM42688P_INT_SOURCE0, INT_SOURCE0_UI_DRDY_INT1_EN);
            if (ret != 0) return ret;
            
            return 0;
        }

        int ICM42688P::set_gyro_range(GyroRange range)
        {
            uint8_t fs_sel;
            
            switch (range) {
            case GyroRange::DPS_125:
                fs_sel = 0x04 << 5;  // 0b100
                update_gyro_scale(range);
                break;
            case GyroRange::DPS_250:
                fs_sel = 0x03 << 5;  // 0b011
                update_gyro_scale(range);
                break;
            case GyroRange::DPS_500:
                fs_sel = 0x02 << 5;  // 0b010
                update_gyro_scale(range);
                break;
            case GyroRange::DPS_1000:
                fs_sel = 0x01 << 5;  // 0b001
                update_gyro_scale(range);
                break;
            case GyroRange::DPS_2000:
                fs_sel = 0x00 << 5;  // 0b000 (default)
                update_gyro_scale(range);
                break;
            default:
                return -EINVAL;
            }
            
            int ret = select_bank(BANK_0);
            if (ret != 0) return ret;
            
            // Read current config to preserve ODR bits
            uint8_t current;
            ret = read_reg(ICM42688P_GYRO_CONFIG0, current);
            if (ret != 0) return ret;
            
            // Set FS_SEL bits [7:5], preserve ODR bits [3:0]
            uint8_t new_val = fs_sel | (current & 0x0F);
            
            ret = register_write_verified(BANK_0, ICM42688P_GYRO_CONFIG0, new_val);
            if (ret == 0) {
                DRIVER_DEBUG("✓ Gyro range: ±%d dps", static_cast<int>(range));
            }
            
            return ret;
        }

        int ICM42688P::set_accel_range(AccelRange range)
        {
            uint8_t fs_sel;
            
            switch (range) {
            case AccelRange::G2:
                fs_sel = 0x03 << 5;  // 0b011
                update_accel_scale(range);
                break;
            case AccelRange::G4:
                fs_sel = 0x02 << 5;  // 0b010
                update_accel_scale(range);
                break;
            case AccelRange::G8:
                fs_sel = 0x01 << 5;  // 0b001
                update_accel_scale(range);
                break;
            case AccelRange::G16:
                fs_sel = 0x00 << 5;  // 0b000 (default)
                update_accel_scale(range);
                break;
            default:
                return -EINVAL;
            }
            
            int ret = select_bank(BANK_0);
            if (ret != 0) return ret;
            
            uint8_t current;
            ret = read_reg(ICM42688P_ACCEL_CONFIG0, current);
            if (ret != 0) return ret;
            
            uint8_t new_val = fs_sel | (current & 0x0F);
            
            ret = register_write_verified(BANK_0, ICM42688P_ACCEL_CONFIG0, new_val);
            if (ret == 0) {
                DRIVER_DEBUG("✓ Accel range: ±%dg", static_cast<int>(range));
            }
            
            return ret;
        }

        int ICM42688P::set_sample_rate(ODR odr)
        {
            int ret = select_bank(BANK_0);
            if (ret != 0) return ret;
            
            uint8_t odr_val = static_cast<uint8_t>(odr);
            
            // Set gyro ODR
            uint8_t gyro_cfg;
            ret = read_reg(ICM42688P_GYRO_CONFIG0, gyro_cfg);
            if (ret != 0) return ret;
            
            gyro_cfg = (gyro_cfg & 0xF0) | odr_val;
            ret = register_write(BANK_0, ICM42688P_GYRO_CONFIG0, gyro_cfg);
            if (ret != 0) return ret;
            
            // Set accel ODR
            uint8_t accel_cfg;
            ret = read_reg(ICM42688P_ACCEL_CONFIG0, accel_cfg);
            if (ret != 0) return ret;
            
            accel_cfg = (accel_cfg & 0xF0) | odr_val;
            ret = register_write(BANK_0, ICM42688P_ACCEL_CONFIG0, accel_cfg);
            if (ret == 0) {
                const char *rate_names[] = {"?", "32kHz", "16kHz", "8kHz", "4kHz", "2kHz", "1kHz"};
                if (odr_val < 7) {
                    DRIVER_DEBUG("✓ Sample rate: %s", rate_names[odr_val]);
                }
            }
            
            return ret;
        }

        void ICM42688P::update_gyro_scale(GyroRange range)
        {
            float sensitivity;
            
            switch (range) {
            case GyroRange::DPS_125:
                sensitivity = 262.0f;  // LSB/(dps)
                break;
            case GyroRange::DPS_250:
                sensitivity = GYRO_SENSITIVITY_250DPS;
                break;
            case GyroRange::DPS_500:
                sensitivity = GYRO_SENSITIVITY_500DPS;
                break;
            case GyroRange::DPS_1000:
                sensitivity = GYRO_SENSITIVITY_1000DPS;
                break;
            case GyroRange::DPS_2000:
            default:
                sensitivity = GYRO_SENSITIVITY_2000DPS;
                break;
            }
            
            // Convert to rad/s
            constexpr float kPi = 3.14159265358979323846f;
            _gyro_scale = (kPi / 180.0f) / sensitivity;
        }

        void ICM42688P::update_accel_scale(AccelRange range)
        {
            float sensitivity;
            
            switch (range) {
            case AccelRange::G2:
                sensitivity = ACCEL_SENSITIVITY_2G;
                break;
            case AccelRange::G4:
                sensitivity = ACCEL_SENSITIVITY_4G;
                break;
            case AccelRange::G8:
                sensitivity = ACCEL_SENSITIVITY_8G;
                break;
            case AccelRange::G16:
            default:
                sensitivity = ACCEL_SENSITIVITY_16G;
                break;
            }
            
            // Convert to m/s²
            _accel_scale = 9.80665f / sensitivity;
        }

        int ICM42688P::read(Data &data)
        {
            if (!_initialized) {
                return -ENODEV;
            }
            
            int ret = select_bank(BANK_0);
            if (ret != 0) {
                _error_count++;
                return ret;
            }
            
            // Read 14 bytes: TEMP(2) + ACCEL(6) + GYRO(6)
            uint8_t buffer[14];
            ret = read_burst(ICM42688P_TEMP_DATA1, buffer, 14);
            if (ret != 0) {
                _error_count++;
                return ret;
            }
            
            data.timestamp_us = hrt_absolute_time();
            _last_read_time = data.timestamp_us;
            _read_count++;
            
            // Parse temperature (big-endian, 16-bit signed)
            int16_t raw_temp = (static_cast<int16_t>(buffer[0]) << 8) | buffer[1];
            data.temperature = (raw_temp / TEMPERATURE_SENSITIVITY) + TEMPERATURE_OFFSET;
            
            // Parse accelerometer (big-endian, 16-bit signed)
            int16_t raw_accel[3];
            raw_accel[0] = (static_cast<int16_t>(buffer[2]) << 8) | buffer[3];
            raw_accel[1] = (static_cast<int16_t>(buffer[4]) << 8) | buffer[5];
            raw_accel[2] = (static_cast<int16_t>(buffer[6]) << 8) | buffer[7];
            
            // Parse gyroscope (big-endian, 16-bit signed)
            int16_t raw_gyro[3];
            raw_gyro[0] = (static_cast<int16_t>(buffer[8]) << 8) | buffer[9];
            raw_gyro[1] = (static_cast<int16_t>(buffer[10]) << 8) | buffer[11];
            raw_gyro[2] = (static_cast<int16_t>(buffer[12]) << 8) | buffer[13];
            
            // Convert to physical units in sensor frame
            calibration::Vector3f accel_raw(
                raw_accel[0] * _accel_scale,
                raw_accel[1] * _accel_scale,
                raw_accel[2] * _accel_scale
            );
            
            calibration::Vector3f gyro_raw(
                raw_gyro[0] * _gyro_scale,
                raw_gyro[1] * _gyro_scale,
                raw_gyro[2] * _gyro_scale
            );
            
            // Apply calibration (bias, scale, rotation) using sensor_calibration library
            calibration::Vector3f accel_corrected = _accel_cal.correct(accel_raw);
            calibration::Vector3f gyro_corrected = _gyro_cal.correct(gyro_raw);
            
            data.accel[0] = accel_corrected.x;
            data.accel[1] = accel_corrected.y;
            data.accel[2] = accel_corrected.z;
            
            data.gyro[0] = gyro_corrected.x;
            data.gyro[1] = gyro_corrected.y;
            data.gyro[2] = gyro_corrected.z;
            
            return 0;
        }

        int ICM42688P::select_bank(icm42688p_bank_t bank)
        {
            if (_current_bank == bank) {
                return 0;
            }
            
            int ret = write_reg(ICM42688P_REG_BANK_SEL, static_cast<uint8_t>(bank));
            if (ret == 0) {
                _current_bank = bank;
            }
            
            return ret;
        }

        int ICM42688P::register_read(icm42688p_bank_t bank, uint8_t reg, uint8_t &value)
        {
            int ret = select_bank(bank);
            if (ret != 0) return ret;
            
            return read_reg(reg, value);
        }

        int ICM42688P::register_write(icm42688p_bank_t bank, uint8_t reg, uint8_t value)
        {
            int ret = select_bank(bank);
            if (ret != 0) return ret;
            
            return write_reg(reg, value);
        }

        int ICM42688P::register_write_verified(icm42688p_bank_t bank, uint8_t reg, uint8_t value)
        {
            int ret = select_bank(bank);
            if (ret != 0) return ret;
            
            return write_reg_verified(reg, value);
        }

        void ICM42688P::print_status()
        {
            printf("\n=== ICM-42688-P Status ===\n");
            printf("Initialized:  %s\n", _initialized ? "YES" : "NO");
            printf("Current bank: %d\n", _current_bank);
            printf("Read count:   %u\n", _read_count);
            printf("Error count:  %u\n", _error_count);
            printf("Success rate: %.2f%%\n", 
                _read_count > 0 ? (100.0f * (_read_count - _error_count) / _read_count) : 0.0f);
            
            if (_last_read_time > 0) {
                uint64_t elapsed = hrt_absolute_time() - _last_read_time;
                printf("Last read:    %llu us ago\n", elapsed);
            }
            
            // Read current config
            select_bank(BANK_0);
            
            uint8_t whoami, pwr_mgmt, gyro_cfg, accel_cfg;
            if (read_reg(ICM42688P_WHO_AM_I, whoami) == 0) {
                printf("WHO_AM_I:     0x%02X\n", whoami);
            }
            
            if (read_reg(ICM42688P_PWR_MGMT0, pwr_mgmt) == 0) {
                printf("PWR_MGMT0:    0x%02X\n", pwr_mgmt);
            }
            
            if (read_reg(ICM42688P_GYRO_CONFIG0, gyro_cfg) == 0) {
                printf("GYRO_CONFIG:  0x%02X\n", gyro_cfg);
            }
            
            if (read_reg(ICM42688P_ACCEL_CONFIG0, accel_cfg) == 0) {
                printf("ACCEL_CONFIG: 0x%02X\n", accel_cfg);
            }
            
            printf("==========================\n\n");
        }

    } // namespace imu
} // namespace drivers