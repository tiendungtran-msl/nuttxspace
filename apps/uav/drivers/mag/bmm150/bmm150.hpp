/****************************************************************************
 * apps/uav/drivers/mag/bmm150/bmm150.hpp
 *
 * BMM150 3-axis Magnetometer Driver (I2C)
 *
 * TÍNH NĂNG:
 * - Giao tiếp I2C sử dụng NuttX I2C API
 * - Đọc dữ liệu từ trường 3 trục (X, Y, Z) đơn vị µT
 * - Bù nhiệt bằng thuật toán Bosch (trim compensation)
 * - Hỗ trợ 4 preset mode: Low Power, Regular, Enhanced, High Accuracy
 * - Configurable ODR: 2 - 30 Hz
 *
 * THAM KHẢO:
 * - Bosch BMM150 SensorAPI (github.com/boschsensortec/BMM150-Sensor-API)
 * - Datasheet BST-BMM150-DS001-05
 ****************************************************************************/

#pragma once

#include <nuttx/config.h>
#include <nuttx/i2c/i2c_master.h>
#include <stdint.h>

#include "bmm150_reg.h"

namespace drivers {
    namespace mag {

        /**
         * @brief BMM150 Magnetometer Driver
         *
         * Driver cho cảm biến từ trường BMM150 của Bosch.
         * Sử dụng giao tiếp I2C thông qua NuttX I2C master API.
         *
         * Quy trình sử dụng:
         *   1. Tạo instance: BMM150 mag(i2c_dev, addr)
         *   2. Khởi tạo:     mag.initialize()
         *   3. Đọc dữ liệu:  mag.read(data)
         */
        class BMM150
        {
        public:
            /**
             * @brief Dữ liệu từ trường đã bù (compensated)
             *
             * Giá trị mag[0..2] có đơn vị micro-Tesla (µT)
             * - mag[0]: Từ trường trục X
             * - mag[1]: Từ trường trục Y
             * - mag[2]: Từ trường trục Z
             */
            struct Data
            {
                float mag[3];           /**< µT (X, Y, Z) */
                uint64_t timestamp_us;  /**< Timestamp đo */
            };

            /**
             * @brief Preset mode cho BMM150
             *
             * Ảnh hưởng đến số lần lặp đo (repetition) và noise level.
             * Nhiều repetition hơn -> noise thấp hơn, nhưng đo chậm hơn.
             */
            enum class PresetMode : uint8_t
            {
                LOW_POWER,       /**< nXY=3,  nZ=3,  noise ~1.0 µT */
                REGULAR,         /**< nXY=9,  nZ=15, noise ~0.6 µT (mặc định) */
                ENHANCED,        /**< nXY=15, nZ=27, noise ~0.5 µT */
                HIGH_ACCURACY    /**< nXY=47, nZ=83, noise ~0.3 µT */
            };

            /**
             * @brief Output Data Rate (chỉ dùng trong Normal mode)
             */
            enum class DataRate : uint8_t
            {
                HZ_2  = BMM150_ODR_2HZ,
                HZ_6  = BMM150_ODR_6HZ,
                HZ_8  = BMM150_ODR_8HZ,
                HZ_10 = BMM150_ODR_10HZ,   /**< Mặc định */
                HZ_15 = BMM150_ODR_15HZ,
                HZ_20 = BMM150_ODR_20HZ,
                HZ_25 = BMM150_ODR_25HZ,
                HZ_30 = BMM150_ODR_30HZ
            };

            /**
             * @brief Constructor
             *
             * @param i2c      Con trỏ tới I2C master device (từ NuttX)
             * @param address  Địa chỉ I2C 7-bit của BMM150 (mặc định 0x13)
             */
            BMM150(struct i2c_master_s *i2c,
                   uint8_t address = BMM150_I2C_ADDR_DEFAULT);

            ~BMM150();

            /**
             * @brief Khởi tạo sensor
             *
             * Quy trình:
             * 1. Bật power control (suspend -> sleep)
             * 2. Đọc và xác nhận Chip ID (0x32)
             * 3. Soft reset
             * 4. Đọc trim registers (hệ số bù)
             * 5. Cấu hình Regular preset mode
             * 6. Đặt Normal operation mode, ODR 20Hz
             *
             * @return 0 nếu thành công, -errno nếu lỗi
             */
            int initialize();

            /**
             * @brief Đọc dữ liệu từ trường đã bù
             *
             * Đọc raw data từ sensor, áp dụng thuật toán bù Bosch
             * để tính giá trị từ trường chính xác (µT).
             *
             * @param data  Con trỏ đến cấu trúc chứa kết quả
             * @return 0 nếu thành công, -errno nếu lỗi
             */
            int read(Data &data);

            /**
             * @brief Đặt preset mode
             *
             * @param mode  Chế độ preset (Low Power/Regular/Enhanced/High Accuracy)
             * @return 0 nếu thành công
             */
            int set_preset_mode(PresetMode mode);

            /**
             * @brief Đặt tốc độ đo (ODR)
             *
             * Chỉ có hiệu lực trong Normal operation mode.
             *
             * @param rate  Tốc độ đo mong muốn
             * @return 0 nếu thành công
             */
            int set_data_rate(DataRate rate);

            /**
             * @brief Đặt operation mode
             *
             * @param mode  BMM150_OPMODE_NORMAL, BMM150_OPMODE_FORCED, hoặc BMM150_OPMODE_SLEEP
             * @return 0 nếu thành công
             */
            int set_op_mode(uint8_t mode);

            /**
             * @brief Kiểm tra sensor đã khởi tạo chưa
             */
            bool is_initialized() const { return _initialized; }

            /**
             * @brief In trạng thái sensor (debug)
             */
            void print_status();

        private:
            struct i2c_master_s *_i2c;      /**< I2C master device */
            uint8_t              _addr;     /**< Địa chỉ I2C 7-bit */
            bool                 _initialized;
            struct bmm150_trim_data_t _trim; /**< Hệ số bù từ factory */

            /* Thống kê */
            uint32_t _read_count;
            uint32_t _error_count;

            /**
             * @brief Đọc một thanh ghi
             */
            int read_reg(uint8_t reg, uint8_t &value);

            /**
             * @brief Đọc nhiều thanh ghi liên tiếp
             */
            int read_regs(uint8_t reg, uint8_t *buffer, size_t len);

            /**
             * @brief Ghi một thanh ghi
             */
            int write_reg(uint8_t reg, uint8_t value);

            /**
             * @brief Soft reset sensor
             */
            int soft_reset();

            /**
             * @brief Đọc trim registers (hệ số bù factory)
             *
             * Đọc các thanh ghi trim từ 0x5D đến 0x71 và lưu vào _trim.
             * Các giá trị này cần thiết cho thuật toán bù Bosch.
             */
            int read_trim_registers();

            /**
             * @brief Thuật toán bù trục X (Bosch compensation)
             *
             * Công thức bù sử dụng Hall resistance value và các
             * hệ số trim để tính giá trị từ trường chính xác.
             *
             * @param raw_x   Giá trị thô trục X (13-bit signed)
             * @param rhall   Giá trị Hall resistance (14-bit unsigned)
             * @return Giá trị đã bù (µT * 16, fixed-point)
             */
            float compensate_x(int16_t raw_x, uint16_t rhall);

            /**
             * @brief Thuật toán bù trục Y (Bosch compensation)
             */
            float compensate_y(int16_t raw_y, uint16_t rhall);

            /**
             * @brief Thuật toán bù trục Z (Bosch compensation)
             */
            float compensate_z(int16_t raw_z, uint16_t rhall);
        };

    } /* namespace mag */
} /* namespace drivers */
