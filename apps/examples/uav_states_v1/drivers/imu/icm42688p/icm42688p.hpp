/****************************************************************************
 * apps/examples/uav_states_v1/drivers/imu/icm42688p/icm42688p.hpp
 *
 * Adapted from PX4 ICM-42688-P driver for Pure NuttX
 ****************************************************************************/

#pragma once

#include "../../../lib/drivers/spi/spi_device.hpp"
#include "../../../lib/drivers_framework/device_id.hpp"
#include "../../../calibration/sensor_calibration.hpp"
#include "icm42688p_regs.h"

namespace drivers {
    namespace imu {

        class ICM42688P : public spi::Device {
        public:
            /** Sensor data */
            struct Data {
                float accel[3];         // m/s² (X, Y, Z)
                float gyro[3];          // rad/s (X, Y, Z)
                float temperature;      // °C
                uint64_t timestamp_us;
            };
            
            /** Gyro range */
            enum class GyroRange : uint16_t {
                DPS_125  = 125,
                DPS_250  = 250,
                DPS_500  = 500,
                DPS_1000 = 1000,
                DPS_2000 = 2000
            };
            
            /** Accel range */
            enum class AccelRange : uint8_t {
                G2  = 2,
                G4  = 4,
                G8  = 8,
                G16 = 16
            };
            
            /** Output Data Rate */
            enum class ODR : uint8_t {
                HZ_1000  = 6,   // 0b0110
                HZ_8000  = 3,   // 0b0011  
                HZ_16000 = 2,   // 0b0010
                HZ_32000 = 1    // 0b0001
            };
            
            ICM42688P(uint8_t bus, uint32_t cs);
            ~ICM42688P();
            
            int initialize();
            int read(Data &data);
            int set_gyro_range(GyroRange range);
            int set_accel_range(AccelRange range);
            int set_sample_rate(ODR odr);

            /**
             * Calibration API using sensor_calibration library
             */
            calibration::Accelerometer& get_accel_calibration() { return _accel_cal; }
            calibration::Gyroscope& get_gyro_calibration() { return _gyro_cal; }
            const calibration::Accelerometer& get_accel_calibration() const { return _accel_cal; }
            const calibration::Gyroscope& get_gyro_calibration() const { return _gyro_cal; }

            /**
             * Legacy API for backward compatibility
             * (Directly modifies internal calibration objects)
             */
            void set_gyro_bias(const float bias_rad_s[3]);
            void set_accel_bias(const float bias_m_s2[3]);
            void set_accel_scale_correction(float scale);

            void get_gyro_bias(float bias_rad_s[3]) const;
            void get_accel_bias(float bias_m_s2[3]) const;
            float get_accel_scale_correction() const;
            
            /** Reset sensor */
            int reset();

            /**
             * Debug helper: read register in a specific bank (read-back config).
             * Restores the previously selected bank.
             */
            int debug_read_reg(icm42688p_bank_t bank, uint8_t reg, uint8_t &value);
            
            /** Print status */
            void print_status();
            
        protected:
            int probe() override;
            
        private:
            float _gyro_scale;
            float _accel_scale;
            
            // Calibration objects
            calibration::Accelerometer _accel_cal;
            calibration::Gyroscope _gyro_cal;
            
            icm42688p_bank_t _current_bank;
            bool _initialized;
            
            // Register access with banking
            int select_bank(icm42688p_bank_t bank);
            int register_read(icm42688p_bank_t bank, uint8_t reg, uint8_t &value);
            int register_write(icm42688p_bank_t bank, uint8_t reg, uint8_t value);
            int register_write_verified(icm42688p_bank_t bank, uint8_t reg, uint8_t value);
            
            // Configuration
            int soft_reset();
            int configure();
            int configure_fifo();
            int configure_interrupt();
            
            // Sensor scale calculation
            void update_gyro_scale(GyroRange range);
            void update_accel_scale(AccelRange range);
            
            // Statistics
            uint32_t _read_count;
            uint32_t _error_count;
            uint64_t _last_read_time;
        };

    } // namespace imu
} // namespace drivers