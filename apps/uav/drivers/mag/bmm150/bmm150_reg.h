/****************************************************************************
 * apps/uav/drivers/mag/bmm150/bmm150_reg.h
 *
 * Định nghĩa thanh ghi cho BMM150 3-axis Magnetometer
 * Tham khảo: Bosch BMM150 SensorAPI (github.com/boschsensortec/BMM150-Sensor-API)
 * Datasheet: BST-BMM150-DS001-05 Rev 1.4
 *
 * BMM150 là cảm biến từ trường 3 trục (geomagnetic sensor)
 * - Giao tiếp: I2C (hoặc SPI)
 * - Độ phân giải: ~0.3 µT (typical)
 * - Dải đo: ±1300 µT (X,Y), ±2500 µT (Z)
 * - ODR: 2 - 30 Hz
 ****************************************************************************/

#ifndef __APPS_UAV_DRIVERS_MAG_BMM150_REG_H
#define __APPS_UAV_DRIVERS_MAG_BMM150_REG_H

#include <stdint.h>

/****************************************************************************
 * I2C Address
 *
 * BMM150 có 4 địa chỉ I2C khả dụng, chọn bằng chân CSB và SDO:
 *   CSB=1, SDO=0 -> 0x10
 *   CSB=1, SDO=1 -> 0x11
 *   CSB=0, SDO=0 -> 0x12
 *   CSB=0, SDO=1 -> 0x13 (mặc định cho module breakout)
 ****************************************************************************/

#define BMM150_I2C_ADDR_DEFAULT         0x13
#define BMM150_I2C_ADDR_CSB1_SDO0       0x10
#define BMM150_I2C_ADDR_CSB1_SDO1       0x11
#define BMM150_I2C_ADDR_CSB0_SDO0       0x12
#define BMM150_I2C_ADDR_CSB0_SDO1       0x13

/****************************************************************************
 * Chip ID
 *
 * Thanh ghi 0x40 chứa ID chip.
 * LƯU Ý: Trong suspend mode, đọc chip_id sẽ trả về 0x00.
 * Cần bật power control bit trước khi đọc chip_id.
 ****************************************************************************/

#define BMM150_REG_CHIP_ID              0x40
#define BMM150_CHIP_ID_VALUE            0x32

/****************************************************************************
 * Data Registers (Read-Only)
 *
 * Dữ liệu từ trường thô (raw) cho 3 trục X, Y, Z
 * và giá trị Hall resistance (R) dùng để bù nhiệt.
 *
 * Cấu trúc dữ liệu (little-endian):
 *   X: 13-bit signed, LSB[4:0] ở 0x42[7:3], MSB[12:5] ở 0x43
 *   Y: 13-bit signed, LSB[4:0] ở 0x44[7:3], MSB[12:5] ở 0x45
 *   Z: 15-bit signed, LSB[6:0] ở 0x46[7:1], MSB[14:7] ở 0x47
 *   R: 14-bit unsigned, LSB[5:0] ở 0x48[7:2], MSB[13:6] ở 0x49
 *
 * Bit 0 của 0x42 là DRDY (Data Ready) status
 ****************************************************************************/

#define BMM150_REG_DATA_X_LSB           0x42
#define BMM150_REG_DATA_X_MSB           0x43
#define BMM150_REG_DATA_Y_LSB           0x44
#define BMM150_REG_DATA_Y_MSB           0x45
#define BMM150_REG_DATA_Z_LSB           0x46
#define BMM150_REG_DATA_Z_MSB           0x47
#define BMM150_REG_RHALL_LSB            0x48
#define BMM150_REG_RHALL_MSB            0x49

/* Số byte cần đọc cho burst read (X_LSB -> RHALL_MSB) */
#define BMM150_DATA_LEN                 8

/* DRDY status bit trong DATA_X_LSB */
#define BMM150_DRDY_STATUS_MSK          0x01

/****************************************************************************
 * Interrupt Status Register (0x4A)
 ****************************************************************************/

#define BMM150_REG_INT_STATUS           0x4A

/****************************************************************************
 * Power Control Register (0x4B)
 *
 * Bit 0: Power control bit
 *   0 = Suspend mode (mặc định sau reset)
 *   1 = Active (Sleep/Normal/Forced mode)
 *
 * Bit 1: Soft reset (ghi 1 để reset)
 * Bit 7:6: Soft reset trigger (ghi 0x82 = soft reset)
 *
 * Quy trình khởi động:
 *   1. Ghi 0x01 vào 0x4B (bật power control)
 *   2. Chờ 3ms (startup time)
 *   3. Đọc chip_id để xác nhận
 *   4. Cấu hình operation mode
 ****************************************************************************/

#define BMM150_REG_POWER_CONTROL        0x4B

#define BMM150_POWER_CONTROL_BIT_MSK    0x01
#define BMM150_POWER_CONTROL_BIT_POS    0

#define BMM150_SOFT_RESET_VALUE         0x82  /* Ghi giá trị này để soft reset */

/****************************************************************************
 * Operation Mode Control Register (0x4C)
 *
 * Bit[2:1]: Opmode
 *   00 = Normal mode  (đo liên tục theo ODR)
 *   01 = Forced mode  (đo 1 lần rồi về sleep)
 *   10 = Forced mode  (tương tự)
 *   11 = Sleep mode   (không đo, tiêu thụ thấp)
 *
 * Bit[5:3]: ODR (Output Data Rate) cho Normal mode
 *   000 = 10 Hz (mặc định)
 *   001 = 2 Hz
 *   010 = 6 Hz
 *   011 = 8 Hz
 *   100 = 15 Hz
 *   101 = 20 Hz
 *   110 = 25 Hz
 *   111 = 30 Hz
 *
 * Bit 6: Self-test
 * Bit 7: Advanced self-test
 ****************************************************************************/

#define BMM150_REG_OP_MODE              0x4C

/* Operation mode bits */
#define BMM150_OP_MODE_MSK              0x06
#define BMM150_OP_MODE_POS              1

#define BMM150_OPMODE_NORMAL            0x00
#define BMM150_OPMODE_FORCED            0x01
#define BMM150_OPMODE_SLEEP             0x03

/* ODR (Data Rate) bits */
#define BMM150_ODR_MSK                  0x38
#define BMM150_ODR_POS                  3

#define BMM150_ODR_10HZ                 0x00  /* Mặc định */
#define BMM150_ODR_2HZ                  0x01
#define BMM150_ODR_6HZ                  0x02
#define BMM150_ODR_8HZ                  0x03
#define BMM150_ODR_15HZ                 0x04
#define BMM150_ODR_20HZ                 0x05
#define BMM150_ODR_25HZ                 0x06
#define BMM150_ODR_30HZ                 0x07

/* Self-test bits */
#define BMM150_SELF_TEST_MSK            0x40
#define BMM150_SELF_TEST_POS            6

#define BMM150_ADV_SELF_TEST_MSK        0xC0
#define BMM150_ADV_SELF_TEST_POS        6

/****************************************************************************
 * Interrupt and Axis Enable Register (0x4D - 0x4F)
 ****************************************************************************/

#define BMM150_REG_INT_CONFIG           0x4D
#define BMM150_REG_INT_AXES_EN          0x4E

/* Interrupt polarity và latch */
#define BMM150_INT_POLARITY_MSK         0x01
#define BMM150_INT_LATCH_MSK            0x02
#define BMM150_DRDY_EN_MSK             0x80

/****************************************************************************
 * Repetition Control Registers (0x51, 0x52)
 *
 * Số lần lặp đo lường cho mỗi trục.
 * Nhiều rep hơn -> noise thấp hơn, nhưng thời gian đo dài hơn.
 *
 * nXY = 1 + 2 * REP_XY  (số lần đo cho trục X,Y)
 * nZ  = 1 + REP_Z       (số lần đo cho trục Z)
 *
 * Preset modes (theo Bosch recommendation):
 *   Low Power:      REP_XY=1,  REP_Z=2     -> nXY=3,  nZ=3
 *   Regular:        REP_XY=4,  REP_Z=14    -> nXY=9,  nZ=15
 *   Enhanced:       REP_XY=7,  REP_Z=26    -> nXY=15, nZ=27
 *   High Accuracy:  REP_XY=23, REP_Z=41    -> nXY=47, nZ=83
 ****************************************************************************/

#define BMM150_REG_REP_XY               0x51
#define BMM150_REG_REP_Z                0x52

/* Preset mode repetition values */
#define BMM150_REPXY_LOWPOWER           0x01
#define BMM150_REPXY_REGULAR            0x04
#define BMM150_REPXY_ENHANCED           0x07
#define BMM150_REPXY_HIGHACCURACY       0x17

#define BMM150_REPZ_LOWPOWER            0x02
#define BMM150_REPZ_REGULAR             0x0E
#define BMM150_REPZ_ENHANCED            0x1A
#define BMM150_REPZ_HIGHACCURACY        0x29

/****************************************************************************
 * Trim Registers (Factory Calibration)
 *
 * Các thanh ghi chứa hệ số bù (compensation coefficients) được
 * lập trình tại nhà máy. Dùng trong thuật toán bù để tính
 * giá trị từ trường chính xác từ dữ liệu thô.
 *
 * Trim data KHÔNG bị reset khi soft reset.
 ****************************************************************************/

/* Trim data registers */
#define BMM150_REG_DIG_X1               0x5D
#define BMM150_REG_DIG_Y1               0x5E
#define BMM150_REG_DIG_Z4_LSB           0x62
#define BMM150_REG_DIG_Z4_MSB           0x63
#define BMM150_REG_DIG_X2               0x64
#define BMM150_REG_DIG_Y2               0x65
#define BMM150_REG_DIG_Z2_LSB           0x68
#define BMM150_REG_DIG_Z2_MSB           0x69
#define BMM150_REG_DIG_Z1_LSB           0x6A
#define BMM150_REG_DIG_Z1_MSB           0x6B
#define BMM150_REG_DIG_XYZ1_LSB         0x6C
#define BMM150_REG_DIG_XYZ1_MSB         0x6D
#define BMM150_REG_DIG_Z3_LSB           0x6E
#define BMM150_REG_DIG_Z3_MSB           0x6F
#define BMM150_REG_DIG_XY2              0x70
#define BMM150_REG_DIG_XY1              0x71

/****************************************************************************
 * Overflow Detection
 *
 * Giá trị trả về khi sensor bị overflow (bão hòa từ trường)
 * X, Y overflow: raw value = -4096
 * Z overflow:    raw value = -16384
 *
 * Output khi overflow:
 *   Nếu dùng integer: BMM150_OVERFLOW_OUTPUT
 *   Nếu dùng float:   BMM150_OVERFLOW_OUTPUT_FLOAT
 ****************************************************************************/

#define BMM150_OVERFLOW_ADCVAL_XYAXES_FLIP  (-4096)
#define BMM150_OVERFLOW_ADCVAL_ZAXIS_HALL   (-16384)
#define BMM150_OVERFLOW_OUTPUT              (-32768)
#define BMM150_OVERFLOW_OUTPUT_FLOAT        0.0f

/* Giá trị positive/negative saturation (bão hòa) cho self-test */
#define BMM150_POSITIVE_SATURATION_Z        32767
#define BMM150_NEGATIVE_SATURATION_Z        (-32767)

/****************************************************************************
 * Timing Constants (microseconds)
 *
 * BMM150 cần các khoảng chờ nhất định khi chuyển chế độ
 ****************************************************************************/

#define BMM150_STARTUP_TIME_US          3000    /* Từ suspend -> sleep: 3ms */
#define BMM150_SOFT_RESET_DELAY_US      5000    /* Chờ sau soft reset: 5ms */
#define BMM150_NORMAL_SELF_TEST_DELAY   500     /* Self-test delay */
#define BMM150_ADV_SELF_TEST_DELAY      4000    /* Advanced self-test delay */

/****************************************************************************
 * Trim Data Structure
 *
 * Cấu trúc lưu trữ các hệ số trim đọc từ sensor.
 * Dùng trong thuật toán bù Bosch để tính giá trị từ trường.
 ****************************************************************************/

struct bmm150_trim_data_t
{
    int8_t   dig_x1;      /* Hệ số bù X1 */
    int8_t   dig_y1;      /* Hệ số bù Y1 */
    int8_t   dig_x2;      /* Hệ số bù X2 */
    int8_t   dig_y2;      /* Hệ số bù Y2 */
    uint16_t dig_z1;      /* Hệ số bù Z1 (unsigned) */
    int16_t  dig_z2;      /* Hệ số bù Z2 */
    int16_t  dig_z3;      /* Hệ số bù Z3 */
    int16_t  dig_z4;      /* Hệ số bù Z4 */
    uint8_t  dig_xy1;     /* Hệ số bù XY1 (unsigned) */
    int8_t   dig_xy2;     /* Hệ số bù XY2 */
    uint16_t dig_xyz1;    /* Hệ số bù XYZ1 (unsigned) */
};

/****************************************************************************
 * Bit Manipulation Macros
 *
 * Macro tiện ích để đọc/ghi các bit field trong thanh ghi
 ****************************************************************************/

#define BMM150_SET_BITS(reg_data, bitname, data) \
    ((reg_data & ~(bitname##_MSK)) | \
     (((data) << bitname##_POS) & bitname##_MSK))

#define BMM150_GET_BITS(reg_data, bitname) \
    ((reg_data & (bitname##_MSK)) >> bitname##_POS)

#endif /* __APPS_UAV_DRIVERS_MAG_BMM150_REG_H */
