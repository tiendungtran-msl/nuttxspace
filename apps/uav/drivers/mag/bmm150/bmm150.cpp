/****************************************************************************
 * apps/uav/drivers/mag/bmm150/bmm150.cpp
 *
 * BMM150 3-axis Magnetometer Driver Implementation
 *
 * THUẬT TOÁN BÙ (COMPENSATION):
 *
 * BMM150 sử dụng Hall sensor để đo từ trường. Giá trị thô cần được
 * bù bằng các hệ số trim (factory calibration) để có kết quả chính xác.
 *
 * Công thức bù cho trục X (tương tự Y):
 *   compensated_x = (raw_x - (dig_xyz1 * 16384 / rhall - 16384)) ...
 *
 * Trục Z có công thức bù khác vì sử dụng sensor cell khác.
 *
 * Tham khảo: Bosch BMM150 SensorAPI - bmm150.c
 ****************************************************************************/

#include "bmm150.hpp"

#include <nuttx/config.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <math.h>

/**
 * Nếu có hrt (high-resolution timer), dùng nó cho timestamp.
 * Nếu không, dùng clock_systime_ticks() * interval.
 */
#ifdef CONFIG_CLOCK_MONOTONIC
#  include <time.h>
static uint64_t _get_timestamp_us()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}
#else
static uint64_t _get_timestamp_us()
{
    return 0;
}
#endif

namespace drivers {
    namespace mag {

        /****************************************************************************
         * Constructor / Destructor
         ****************************************************************************/

        BMM150::BMM150(struct i2c_master_s *i2c, uint8_t address)
            : _i2c(i2c)
            , _addr(address)
            , _initialized(false)
            , _read_count(0)
            , _error_count(0)
        {
            memset(&_trim, 0, sizeof(_trim));
        }

        BMM150::~BMM150()
        {
        }

        /****************************************************************************
         * I2C Register Access
         *
         * Sử dụng NuttX I2C_TRANSFER() API cho giao tiếp I2C.
         * Mỗi transaction gồm 2 message:
         *   1. Ghi địa chỉ thanh ghi (1 byte)
         *   2. Đọc dữ liệu (n bytes)
         ****************************************************************************/

        int BMM150::read_reg(uint8_t reg, uint8_t &value)
        {
            struct i2c_msg_s msgs[2];

            /* Message 1: Ghi địa chỉ thanh ghi */
            msgs[0].frequency = 400000;        /* 400 kHz Fast-mode I2C */
            msgs[0].addr      = _addr;
            msgs[0].flags     = 0;             /* Write */
            msgs[0].buffer    = &reg;
            msgs[0].length    = 1;

            /* Message 2: Đọc 1 byte dữ liệu */
            msgs[1].frequency = 400000;
            msgs[1].addr      = _addr;
            msgs[1].flags     = I2C_M_READ;
            msgs[1].buffer    = &value;
            msgs[1].length    = 1;

            int ret = I2C_TRANSFER(_i2c, msgs, 2);
            if (ret < 0)
            {
                return ret;
            }

            return 0;
        }

        int BMM150::read_regs(uint8_t reg, uint8_t *buffer, size_t len)
        {
            struct i2c_msg_s msgs[2];

            /* Message 1: Ghi địa chỉ thanh ghi bắt đầu */
            msgs[0].frequency = 400000;
            msgs[0].addr      = _addr;
            msgs[0].flags     = 0;
            msgs[0].buffer    = &reg;
            msgs[0].length    = 1;

            /* Message 2: Đọc nhiều byte liên tiếp (auto-increment) */
            msgs[1].frequency = 400000;
            msgs[1].addr      = _addr;
            msgs[1].flags     = I2C_M_READ;
            msgs[1].buffer    = buffer;
            msgs[1].length    = len;

            int ret = I2C_TRANSFER(_i2c, msgs, 2);
            if (ret < 0)
            {
                return ret;
            }

            return 0;
        }

        int BMM150::write_reg(uint8_t reg, uint8_t value)
        {
            uint8_t buffer[2];

            buffer[0] = reg;
            buffer[1] = value;

            struct i2c_msg_s msg;
            msg.frequency = 400000;
            msg.addr      = _addr;
            msg.flags     = 0;            /* Write */
            msg.buffer    = buffer;
            msg.length    = 2;

            int ret = I2C_TRANSFER(_i2c, &msg, 1);
            if (ret < 0)
            {
                return ret;
            }

            return 0;
        }

        /****************************************************************************
         * Initialization
         *
         * Quy trình khởi tạo BMM150:
         *
         * 1. POWER ON: Ghi 0x01 vào Power Control Register (0x4B)
         *    -> Chuyển từ Suspend mode sang Sleep mode
         *    -> Chờ 3ms (startup time)
         *
         * 2. CHIP ID: Đọc thanh ghi 0x40, phải trả về 0x32
         *    -> Nếu sai -> sensor không phải BMM150 hoặc bị lỗi
         *
         * 3. SOFT RESET: Ghi 0x82 vào 0x4B
         *    -> Reset tất cả thanh ghi (trừ trim và power control)
         *    -> Chờ 5ms
         *    -> Bật lại power control
         *
         * 4. TRIM DATA: Đọc 16 thanh ghi trim (0x5D - 0x71)
         *    -> Lưu vào _trim structure
         *    -> Cần thiết cho thuật toán bù
         *
         * 5. CONFIGURE: Đặt preset mode Regular, ODR 20Hz, Normal mode
         ****************************************************************************/

        int BMM150::initialize()
        {
            int ret;

            if (_i2c == nullptr)
            {
                printf("[BMM150] I2C device is NULL\n");
                return -ENODEV;
            }

            /* Bước 1: Bật power control (exit suspend mode) */
            ret = write_reg(BMM150_REG_POWER_CONTROL, 0x01);
            if (ret < 0)
            {
                printf("[BMM150] Failed to enable power control: %d\n", ret);
                return ret;
            }

            /* Chờ startup time (suspend -> sleep) */
            usleep(BMM150_STARTUP_TIME_US);

            /* Bước 2: Đọc và xác nhận Chip ID */
            uint8_t chip_id = 0;
            ret = read_reg(BMM150_REG_CHIP_ID, chip_id);
            if (ret < 0)
            {
                printf("[BMM150] Failed to read chip ID: %d\n", ret);
                return ret;
            }

            if (chip_id != BMM150_CHIP_ID_VALUE)
            {
                printf("[BMM150] Wrong Chip ID: 0x%02X (expected 0x%02X)\n",
                       chip_id, BMM150_CHIP_ID_VALUE);
                return -ENODEV;
            }

            printf("[BMM150] Chip ID OK: 0x%02X\n", chip_id);

            /* Bước 3: Soft reset */
            ret = soft_reset();
            if (ret < 0)
            {
                printf("[BMM150] Soft reset failed: %d\n", ret);
                return ret;
            }

            /* Bước 4: Đọc trim registers */
            ret = read_trim_registers();
            if (ret < 0)
            {
                printf("[BMM150] Failed to read trim registers: %d\n", ret);
                return ret;
            }

            /* Bước 5: Cấu hình sensor */

            /* Đặt preset mode Regular */
            ret = set_preset_mode(PresetMode::REGULAR);
            if (ret < 0)
            {
                printf("[BMM150] Failed to set preset mode: %d\n", ret);
                return ret;
            }

            /* Đặt ODR 20Hz */
            ret = set_data_rate(DataRate::HZ_20);
            if (ret < 0)
            {
                printf("[BMM150] Failed to set data rate: %d\n", ret);
                return ret;
            }

            /* Đặt Normal operation mode */
            ret = set_op_mode(BMM150_OPMODE_NORMAL);
            if (ret < 0)
            {
                printf("[BMM150] Failed to set op mode: %d\n", ret);
                return ret;
            }

            _initialized = true;
            printf("[BMM150] Initialized successfully (Regular mode, 20 Hz)\n");

            return 0;
        }

        /****************************************************************************
         * Soft Reset
         *
         * Ghi giá trị soft reset (0x82) vào Power Control Register.
         * Thanh ghi này đặc biệt: giá trị power control bit được giữ lại
         * sau reset, nhưng tất cả thanh ghi cấu hình khác sẽ reset.
         *
         * Sau reset, cần bật lại power control để truy cập sensor.
         ****************************************************************************/

        int BMM150::soft_reset()
        {
            /* Ghi soft reset value */
            int ret = write_reg(BMM150_REG_POWER_CONTROL, BMM150_SOFT_RESET_VALUE);
            if (ret < 0)
            {
                return ret;
            }

            usleep(BMM150_SOFT_RESET_DELAY_US);

            /* Bật lại power control sau reset */
            ret = write_reg(BMM150_REG_POWER_CONTROL, 0x01);
            if (ret < 0)
            {
                return ret;
            }

            usleep(BMM150_STARTUP_TIME_US);

            return 0;
        }

        /****************************************************************************
         * Read Trim Registers
         *
         * Đọc hệ số bù factory (trim data) từ các thanh ghi không liên tục:
         *   0x5D: dig_x1 (int8)
         *   0x5E: dig_y1 (int8)
         *   0x62-0x63: dig_z4 (int16, little-endian)
         *   0x64: dig_x2 (int8)
         *   0x65: dig_y2 (int8)
         *   0x68-0x69: dig_z2 (int16, little-endian)
         *   0x6A-0x6B: dig_z1 (uint16, little-endian)
         *   0x6C-0x6D: dig_xyz1 (uint16, little-endian)
         *   0x6E-0x6F: dig_z3 (int16, little-endian)
         *   0x70: dig_xy2 (int8)
         *   0x71: dig_xy1 (uint8)
         *
         * Các thanh ghi này nằm rải rác nên cần đọc theo nhóm.
         ****************************************************************************/

        int BMM150::read_trim_registers()
        {
            uint8_t trim_x1y1[2];
            uint8_t trim_xyz_data[4];
            uint8_t trim_xy1xy2[10];
            int ret;

            /* Nhóm 1: dig_x1 (0x5D), dig_y1 (0x5E) */
            ret = read_regs(BMM150_REG_DIG_X1, trim_x1y1, 2);
            if (ret < 0) return ret;

            _trim.dig_x1 = (int8_t)trim_x1y1[0];
            _trim.dig_y1 = (int8_t)trim_x1y1[1];

            /* Nhóm 2: dig_z4 (0x62-0x63), dig_x2 (0x64), dig_y2 (0x65) */
            ret = read_regs(BMM150_REG_DIG_Z4_LSB, trim_xyz_data, 4);
            if (ret < 0) return ret;

            _trim.dig_z4 = (int16_t)((uint16_t)trim_xyz_data[1] << 8 |
                                      trim_xyz_data[0]);
            _trim.dig_x2 = (int8_t)trim_xyz_data[2];
            _trim.dig_y2 = (int8_t)trim_xyz_data[3];

            /* Nhóm 3: dig_z2 (0x68-0x69), dig_z1 (0x6A-0x6B),
             *          dig_xyz1 (0x6C-0x6D), dig_z3 (0x6E-0x6F),
             *          dig_xy2 (0x70), dig_xy1 (0x71) */
            ret = read_regs(BMM150_REG_DIG_Z2_LSB, trim_xy1xy2, 10);
            if (ret < 0) return ret;

            _trim.dig_z2   = (int16_t)((uint16_t)trim_xy1xy2[1] << 8 |
                                        trim_xy1xy2[0]);
            _trim.dig_z1   = (uint16_t)((uint16_t)trim_xy1xy2[3] << 8 |
                                         trim_xy1xy2[2]);
            _trim.dig_xyz1 = (uint16_t)((uint16_t)trim_xy1xy2[5] << 8 |
                                         trim_xy1xy2[4]);
            _trim.dig_z3   = (int16_t)((uint16_t)trim_xy1xy2[7] << 8 |
                                        trim_xy1xy2[6]);
            _trim.dig_xy2  = (int8_t)trim_xy1xy2[8];
            _trim.dig_xy1  = trim_xy1xy2[9];

            return 0;
        }

        /****************************************************************************
         * Thuật toán bù Bosch (Compensation Algorithm)
         *
         * Chuyển đổi giá trị thô (raw ADC) thành giá trị từ trường (µT)
         * sử dụng các hệ số trim factory calibration.
         *
         * Thuật toán này được lấy trực tiếp từ Bosch BMM150 SensorAPI.
         * Sử dụng floating-point arithmetic cho độ chính xác cao.
         *
         * Tham số đầu vào:
         *   raw_x/y/z: Giá trị ADC thô (đã sign-extend)
         *   rhall:     Hall resistance value (dùng để bù nhiệt)
         *
         * CÔNG THỨC (đơn giản hóa cho trục X):
         *
         *   process_comp_x0 = dig_xyz1 * 16384.0 / rhall - 16384.0
         *   process_comp_x1 = dig_xy2 * (process_comp_x0^2 / 268435456.0)
         *   process_comp_x2 = process_comp_x1 + process_comp_x0 * dig_xy1 / 16384.0
         *   process_comp_x3 = dig_x2 + 160.0
         *   process_comp_x4 = raw_x * ((process_comp_x2 + 256.0) * process_comp_x3)
         *   compensated_x   = process_comp_x4 / 8192.0 + dig_x1 * 8.0
         *                     → đơn vị: µT * 16 (chia 16 để ra µT)
         ****************************************************************************/

        float BMM150::compensate_x(int16_t raw_x, uint16_t rhall)
        {
            float retval = BMM150_OVERFLOW_OUTPUT_FLOAT;

            /* Kiểm tra overflow */
            if (raw_x != BMM150_OVERFLOW_ADCVAL_XYAXES_FLIP)
            {
                /* Kiểm tra rhall hợp lệ */
                if (rhall != 0)
                {
                    float process_comp_x0 = ((float)_trim.dig_xyz1) * 16384.0f
                                            / (float)rhall;
                    retval = process_comp_x0 - 16384.0f;
                }
                else
                {
                    retval = 0.0f;
                }

                float process_comp_x1 = ((float)_trim.dig_xy2) *
                                         (retval * retval / 268435456.0f);
                float process_comp_x2 = process_comp_x1 +
                                         retval * ((float)_trim.dig_xy1) / 16384.0f;
                float process_comp_x3 = ((float)_trim.dig_x2) + 160.0f;
                float process_comp_x4 = (float)raw_x *
                                         ((process_comp_x2 + 256.0f) *
                                          process_comp_x3);

                /* Kết quả: µT * 16 -> chia 8192 rồi cộng offset */
                retval = ((process_comp_x4 / 8192.0f) +
                          (((float)_trim.dig_x1) * 8.0f)) / 16.0f;
            }

            return retval;
        }

        float BMM150::compensate_y(int16_t raw_y, uint16_t rhall)
        {
            float retval = BMM150_OVERFLOW_OUTPUT_FLOAT;

            if (raw_y != BMM150_OVERFLOW_ADCVAL_XYAXES_FLIP)
            {
                if (rhall != 0)
                {
                    float process_comp_y0 = ((float)_trim.dig_xyz1) * 16384.0f
                                            / (float)rhall;
                    retval = process_comp_y0 - 16384.0f;
                }
                else
                {
                    retval = 0.0f;
                }

                float process_comp_y1 = ((float)_trim.dig_xy2) *
                                         (retval * retval / 268435456.0f);
                float process_comp_y2 = process_comp_y1 +
                                         retval * ((float)_trim.dig_xy1) / 16384.0f;
                float process_comp_y3 = ((float)_trim.dig_y2) + 160.0f;
                float process_comp_y4 = (float)raw_y *
                                         ((process_comp_y2 + 256.0f) *
                                          process_comp_y3);

                retval = ((process_comp_y4 / 8192.0f) +
                          (((float)_trim.dig_y1) * 8.0f)) / 16.0f;
            }

            return retval;
        }

        /**
         * @brief Bù trục Z
         *
         * Trục Z có công thức bù khác X/Y vì sử dụng sensor cell khác
         * với đặc tính riêng. Công thức:
         *
         *   compensated_z = (raw_z - dig_z4) * 131072.0 -
         *                   dig_z3 * (rhall - dig_xyz1)
         *                   / (dig_z2 + dig_z1 * rhall / 32768.0 - 131072.0)
         *                   / 16.0
         */
        float BMM150::compensate_z(int16_t raw_z, uint16_t rhall)
        {
            float retval = BMM150_OVERFLOW_OUTPUT_FLOAT;

            if (raw_z != BMM150_OVERFLOW_ADCVAL_ZAXIS_HALL)
            {
                float process_comp_z0 = ((float)raw_z) - ((float)_trim.dig_z4);
                float process_comp_z1 = ((float)rhall) - ((float)_trim.dig_xyz1);
                float process_comp_z2 = (float)_trim.dig_z3 * process_comp_z1;
                float process_comp_z3 = (float)_trim.dig_z1 * ((float)rhall) / 32768.0f;
                float process_comp_z4 = (float)_trim.dig_z2 + process_comp_z3;
                float process_comp_z5 = (process_comp_z0 * 131072.0f - process_comp_z2);

                /* Tránh chia cho 0 */
                if (process_comp_z4 != 0.0f)
                {
                    retval = (process_comp_z5 / ((process_comp_z4) * 4.0f)) / 16.0f;
                }
            }

            return retval;
        }

        /****************************************************************************
         * Read Magnetometer Data
         *
         * Đọc 8 byte dữ liệu (X_LSB -> RHALL_MSB) từ sensor,
         * parse raw values, rồi áp dụng thuật toán bù Bosch.
         *
         * Format dữ liệu raw:
         *   X: 13-bit signed, bits [12:5] ở MSB, bits [4:0] ở LSB[7:3]
         *   Y: 13-bit signed, tương tự X
         *   Z: 15-bit signed, bits [14:7] ở MSB, bits [6:0] ở LSB[7:1]
         *   R: 14-bit unsigned, bits [13:6] ở MSB, bits [5:0] ở LSB[7:2]
         *
         * Bit 0 của X_LSB là Data Ready status bit.
         ****************************************************************************/

        int BMM150::read(Data &data)
        {
            if (!_initialized)
            {
                return -ENODEV;
            }

            uint8_t buffer[BMM150_DATA_LEN];
            int ret = read_regs(BMM150_REG_DATA_X_LSB, buffer, BMM150_DATA_LEN);
            if (ret < 0)
            {
                _error_count++;
                return ret;
            }

            _read_count++;
            data.timestamp_us = _get_timestamp_us();

            /**
             * Parse raw data
             *
             * X raw (13-bit signed):
             *   MSB = buffer[1] (8 bits = bits [12:5])
             *   LSB = buffer[0] >> 3 (5 bits = bits [4:0])
             *   Ghép: (MSB << 5) | (LSB >> 3)
             *   Sign-extend từ 13-bit: chia cho 8 (shift right arithmetic)
             *   → (int16_t)(MSB << 8 | LSB) >> 3
             */
            int16_t raw_x = (int16_t)((uint16_t)buffer[1] << 8 | buffer[0]);
            raw_x >>= 3; /* Sign-extend 13-bit -> 16-bit, align */

            int16_t raw_y = (int16_t)((uint16_t)buffer[3] << 8 | buffer[2]);
            raw_y >>= 3; /* 13-bit signed */

            int16_t raw_z = (int16_t)((uint16_t)buffer[5] << 8 | buffer[4]);
            raw_z >>= 1; /* 15-bit signed */

            uint16_t rhall = (uint16_t)((uint16_t)buffer[7] << 8 | buffer[6]);
            rhall >>= 2; /* 14-bit unsigned */

            /* Áp dụng thuật toán bù Bosch -> kết quả µT */
            data.mag[0] = compensate_x(raw_x, rhall);
            data.mag[1] = compensate_y(raw_y, rhall);
            data.mag[2] = compensate_z(raw_z, rhall);

            return 0;
        }

        /****************************************************************************
         * Configuration
         ****************************************************************************/

        int BMM150::set_preset_mode(PresetMode mode)
        {
            uint8_t rep_xy;
            uint8_t rep_z;

            switch (mode)
            {
            case PresetMode::LOW_POWER:
                rep_xy = BMM150_REPXY_LOWPOWER;
                rep_z  = BMM150_REPZ_LOWPOWER;
                break;

            case PresetMode::REGULAR:
                rep_xy = BMM150_REPXY_REGULAR;
                rep_z  = BMM150_REPZ_REGULAR;
                break;

            case PresetMode::ENHANCED:
                rep_xy = BMM150_REPXY_ENHANCED;
                rep_z  = BMM150_REPZ_ENHANCED;
                break;

            case PresetMode::HIGH_ACCURACY:
                rep_xy = BMM150_REPXY_HIGHACCURACY;
                rep_z  = BMM150_REPZ_HIGHACCURACY;
                break;

            default:
                return -EINVAL;
            }

            int ret = write_reg(BMM150_REG_REP_XY, rep_xy);
            if (ret < 0) return ret;

            ret = write_reg(BMM150_REG_REP_Z, rep_z);
            if (ret < 0) return ret;

            return 0;
        }

        int BMM150::set_data_rate(DataRate rate)
        {
            /* Đọc thanh ghi hiện tại để giữ nguyên các bit khác */
            uint8_t reg_val;
            int ret = read_reg(BMM150_REG_OP_MODE, reg_val);
            if (ret < 0) return ret;

            /* Sửa ODR bits [5:3] */
            reg_val = BMM150_SET_BITS(reg_val, BMM150_ODR,
                                      static_cast<uint8_t>(rate));

            ret = write_reg(BMM150_REG_OP_MODE, reg_val);
            return ret;
        }

        int BMM150::set_op_mode(uint8_t mode)
        {
            /* Đọc thanh ghi hiện tại */
            uint8_t reg_val;
            int ret = read_reg(BMM150_REG_OP_MODE, reg_val);
            if (ret < 0) return ret;

            /* Sửa Opmode bits [2:1] */
            reg_val = BMM150_SET_BITS(reg_val, BMM150_OP_MODE, mode);

            ret = write_reg(BMM150_REG_OP_MODE, reg_val);
            return ret;
        }

        /****************************************************************************
         * Debug
         ****************************************************************************/

        void BMM150::print_status()
        {
            printf("\n=== BMM150 Magnetometer Status ===\n");
            printf("Initialized:  %s\n", _initialized ? "YES" : "NO");
            printf("I2C address:  0x%02X\n", _addr);
            printf("Read count:   %lu\n", (unsigned long)_read_count);
            printf("Error count:  %lu\n", (unsigned long)_error_count);

            if (_initialized)
            {
                /* Đọc trạng thái hiện tại */
                uint8_t chip_id, pwr_ctrl, op_mode, rep_xy, rep_z;

                if (read_reg(BMM150_REG_CHIP_ID, chip_id) == 0)
                {
                    printf("Chip ID:      0x%02X\n", chip_id);
                }

                if (read_reg(BMM150_REG_POWER_CONTROL, pwr_ctrl) == 0)
                {
                    printf("Power Ctrl:   0x%02X (%s)\n", pwr_ctrl,
                           (pwr_ctrl & 0x01) ? "Active" : "Suspend");
                }

                if (read_reg(BMM150_REG_OP_MODE, op_mode) == 0)
                {
                    uint8_t mode = BMM150_GET_BITS(op_mode, BMM150_OP_MODE);
                    uint8_t odr  = BMM150_GET_BITS(op_mode, BMM150_ODR);
                    const char *mode_str;

                    switch (mode)
                    {
                    case BMM150_OPMODE_NORMAL: mode_str = "Normal"; break;
                    case BMM150_OPMODE_FORCED: mode_str = "Forced"; break;
                    case BMM150_OPMODE_SLEEP:  mode_str = "Sleep";  break;
                    default:                   mode_str = "Unknown"; break;
                    }

                    printf("Op Mode:      %s\n", mode_str);
                    printf("ODR setting:  %d\n", odr);
                }

                if (read_reg(BMM150_REG_REP_XY, rep_xy) == 0 &&
                    read_reg(BMM150_REG_REP_Z, rep_z) == 0)
                {
                    printf("Rep XY:       %d (nXY=%d)\n",
                           rep_xy, 1 + 2 * rep_xy);
                    printf("Rep Z:        %d (nZ=%d)\n",
                           rep_z, 1 + rep_z);
                }

                /* Đọc và hiển thị trim data */
                printf("Trim: x1=%d y1=%d x2=%d y2=%d\n",
                       _trim.dig_x1, _trim.dig_y1,
                       _trim.dig_x2, _trim.dig_y2);
                printf("Trim: z1=%u z2=%d z3=%d z4=%d\n",
                       _trim.dig_z1, _trim.dig_z2,
                       _trim.dig_z3, _trim.dig_z4);
                printf("Trim: xy1=%u xy2=%d xyz1=%u\n",
                       _trim.dig_xy1, _trim.dig_xy2, _trim.dig_xyz1);
            }

            printf("==================================\n\n");
        }

    } /* namespace mag */
} /* namespace drivers */
