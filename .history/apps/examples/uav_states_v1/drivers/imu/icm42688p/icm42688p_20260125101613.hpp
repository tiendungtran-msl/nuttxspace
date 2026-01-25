/****************************************************************************
 * apps/examples/uav_states_v1/drivers/imu/icm42688p/icm42688p.hpp
 ****************************************************************************/

#pragma once

#include "../../../lib/drivers/spi/spi_device.hpp"
#include "../../../lib/drivers_framework/device_id.hpp"
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
                HZ_100   = GYRO_CONFIG0_ODR_100HZ,
                HZ_200   = GYRO_CONFIG0_ODR_200HZ,
                HZ_500   = GYRO_CONFIG0_ODR_500HZ,
                HZ_1000  = GYRO_CONFIG0_ODR_1KHZ,
            };
            
            ICM42688P(uint8_t bus, uint32_t cs);
            
            int initialize();
            int read(Data &data);
            int set_gyro_range(GyroRange range);
            int set_accel_range(AccelRange range);
            int set_odr(ODR odr);
            
        protected:
            int probe() override;
            
        private:
            float _gyro_scale;
            float _accel_scale;
            icm42688p_bank_t _current_bank;
            bool _initialized;
            
            int select_bank(icm42688p_bank_t bank);
            int soft_reset();
            int configure();
        };

    } // namespace imu
} // namespace drivers