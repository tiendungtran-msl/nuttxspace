/****************************************************************************
 * apps/uav/drivers/imu/icm42688p.hpp
 *
 * ICM42688P 6-axis IMU Driver
 *
 * Đọc dữ liệu từ ICM42688P qua SPI
 * - Accelerometer: ±16g, 16-bit
 * - Gyroscope: ±2000 dps, 16-bit
 * - Temperature sensor
 *
 ****************************************************************************/

#ifndef __UAV_DRIVERS_IMU_ICM42688P_HPP
#define __UAV_DRIVERS_IMU_ICM42688P_HPP

#include <nuttx/config.h>
#include <nuttx/spi/spi.h>
#include <stdint.h>

/****************************************************************************
 * ICM42688P Register Definitions
 ****************************************************************************/

/* User Bank 0 registers */
#define ICM42688P_REG_DEVICE_CONFIG     0x11
#define ICM42688P_REG_DRIVE_CONFIG      0x13
#define ICM42688P_REG_INT_CONFIG        0x14
#define ICM42688P_REG_FIFO_CONFIG       0x16
#define ICM42688P_REG_TEMP_DATA1        0x1D
#define ICM42688P_REG_TEMP_DATA0        0x1E
#define ICM42688P_REG_ACCEL_DATA_X1     0x1F
#define ICM42688P_REG_ACCEL_DATA_X0     0x20
#define ICM42688P_REG_ACCEL_DATA_Y1     0x21
#define ICM42688P_REG_ACCEL_DATA_Y0     0x22
#define ICM42688P_REG_ACCEL_DATA_Z1     0x23
#define ICM42688P_REG_ACCEL_DATA_Z0     0x24
#define ICM42688P_REG_GYRO_DATA_X1      0x25
#define ICM42688P_REG_GYRO_DATA_X0      0x26
#define ICM42688P_REG_GYRO_DATA_Y1      0x27
#define ICM42688P_REG_GYRO_DATA_Y0      0x28
#define ICM42688P_REG_GYRO_DATA_Z1      0x29
#define ICM42688P_REG_GYRO_DATA_Z0      0x2A
#define ICM42688P_REG_INT_STATUS        0x2D
#define ICM42688P_REG_PWR_MGMT0         0x4E
#define ICM42688P_REG_GYRO_CONFIG0      0x4F
#define ICM42688P_REG_ACCEL_CONFIG0     0x50
#define ICM42688P_REG_GYRO_CONFIG1      0x51
#define ICM42688P_REG_ACCEL_CONFIG1     0x53
#define ICM42688P_REG_WHO_AM_I          0x75
#define ICM42688P_REG_BANK_SEL          0x76

/* WHO_AM_I value */
#define ICM42688P_WHO_AM_I_VALUE        0x47

/* PWR_MGMT0 bits */
#define ICM42688P_GYRO_MODE_LN          (3 << 2)  /* Low noise mode */
#define ICM42688P_ACCEL_MODE_LN         (3 << 0)  /* Low noise mode */

/* Gyro config: ±2000 dps, ODR 1kHz */
#define ICM42688P_GYRO_FS_2000DPS       (0 << 5)
#define ICM42688P_GYRO_ODR_1KHZ         (6 << 0)

/* Accel config: ±16g, ODR 1kHz */
#define ICM42688P_ACCEL_FS_16G          (0 << 5)
#define ICM42688P_ACCEL_ODR_1KHZ        (6 << 0)

/* Scale factors */
#define ICM42688P_GYRO_SCALE_2000DPS    (2000.0f / 32768.0f * 0.0174533f)  /* rad/s per LSB */
#define ICM42688P_ACCEL_SCALE_16G       (16.0f / 32768.0f * 9.80665f)      /* m/s² per LSB */
#define ICM42688P_TEMP_SCALE            (1.0f / 132.48f)
#define ICM42688P_TEMP_OFFSET           25.0f

/****************************************************************************
 * Raw IMU Data Structure
 ****************************************************************************/

struct icm42688p_raw_data_s
{
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t temp;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
};

/****************************************************************************
 * Scaled IMU Data Structure
 ****************************************************************************/

struct icm42688p_data_s
{
    float accel_x;      /* m/s² */
    float accel_y;      /* m/s² */
    float accel_z;      /* m/s² */
    float gyro_x;       /* rad/s */
    float gyro_y;       /* rad/s */
    float gyro_z;       /* rad/s */
    float temperature;  /* °C */
};

/****************************************************************************
 * ICM42688P Driver Class
 ****************************************************************************/

class ICM42688P
{
public:
    ICM42688P(struct spi_dev_s *spi, uint8_t instance);
    ~ICM42688P();

    /**
     * @brief Initialize the sensor
     * @return 0 on success, negative errno on failure
     */
    int init(void);

    /**
     * @brief Check if sensor is initialized and responding
     * @return true if sensor is OK
     */
    bool is_ok(void) const { return m_initialized; }

    /**
     * @brief Read raw sensor data
     * @param data Pointer to raw data structure
     * @return 0 on success, negative errno on failure
     */
    int read_raw(struct icm42688p_raw_data_s *data);

    /**
     * @brief Read scaled sensor data
     * @param data Pointer to scaled data structure
     * @return 0 on success, negative errno on failure
     */
    int read(struct icm42688p_data_s *data);

    /**
     * @brief Get sensor instance number
     */
    uint8_t get_instance(void) const { return m_instance; }

private:
    struct spi_dev_s *m_spi;
    uint8_t m_instance;
    bool m_initialized;

    /**
     * @brief Read a single register
     */
    uint8_t read_reg(uint8_t reg);

    /**
     * @brief Write a single register
     */
    void write_reg(uint8_t reg, uint8_t value);

    /**
     * @brief Read multiple registers
     */
    void read_regs(uint8_t reg, uint8_t *buffer, size_t len);
};

#endif /* __UAV_DRIVERS_IMU_ICM42688P_HPP */
