/****************************************************************************
 * apps/uav/drivers/imu/icm42688p.cpp
 *
 * ICM42688P 6-axis IMU Driver Implementation
 *
 ****************************************************************************/

#include "icm42688p.hpp"

#include <nuttx/config.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <syslog.h>
#include <unistd.h>

/****************************************************************************
 * Constructor/Destructor
 ****************************************************************************/

ICM42688P::ICM42688P(struct spi_dev_s *spi, uint8_t instance)
    : m_spi(spi)
    , m_instance(instance)
    , m_initialized(false)
{
}

ICM42688P::~ICM42688P()
{
}

/****************************************************************************
 * Private Methods
 ****************************************************************************/

uint8_t ICM42688P::read_reg(uint8_t reg)
{
    uint8_t tx[2];
    uint8_t rx[2];

    tx[0] = reg | 0x80;  /* Set read bit */
    tx[1] = 0x00;

    SPI_LOCK(m_spi, true);
    SPI_SELECT(m_spi, SPIDEV_IMU(m_instance), true);
    SPI_EXCHANGE(m_spi, tx, rx, 2);
    SPI_SELECT(m_spi, SPIDEV_IMU(m_instance), false);
    SPI_LOCK(m_spi, false);

    return rx[1];
}

void ICM42688P::write_reg(uint8_t reg, uint8_t value)
{
    uint8_t tx[2];

    tx[0] = reg & 0x7F;  /* Clear read bit */
    tx[1] = value;

    SPI_LOCK(m_spi, true);
    SPI_SELECT(m_spi, SPIDEV_IMU(m_instance), true);
    SPI_EXCHANGE(m_spi, tx, NULL, 2);
    SPI_SELECT(m_spi, SPIDEV_IMU(m_instance), false);
    SPI_LOCK(m_spi, false);
}

void ICM42688P::read_regs(uint8_t reg, uint8_t *buffer, size_t len)
{
    uint8_t tx[16];

    if (len > 15)
    {
        len = 15;
    }

    memset(tx, 0, sizeof(tx));
    tx[0] = reg | 0x80;  /* Set read bit */

    SPI_LOCK(m_spi, true);
    SPI_SELECT(m_spi, SPIDEV_IMU(m_instance), true);
    SPI_EXCHANGE(m_spi, tx, buffer, len + 1);
    SPI_SELECT(m_spi, SPIDEV_IMU(m_instance), false);
    SPI_LOCK(m_spi, false);

    /* Shift data (first byte is dummy) */
    memmove(buffer, buffer + 1, len);
}

/****************************************************************************
 * Public Methods
 ****************************************************************************/

int ICM42688P::init(void)
{
    uint8_t who_am_i;

    if (m_spi == NULL)
    {
        syslog(LOG_ERR, "[ICM42688P-%d] SPI device is NULL\n", m_instance);
        return -ENODEV;
    }

    /* Soft reset */
    write_reg(ICM42688P_REG_DEVICE_CONFIG, 0x01);
    usleep(10000);  /* Wait 10ms for reset */

    /* Check WHO_AM_I */
    who_am_i = read_reg(ICM42688P_REG_WHO_AM_I);
    
    if (who_am_i != ICM42688P_WHO_AM_I_VALUE)
    {
        syslog(LOG_ERR, "[ICM42688P-%d] WHO_AM_I mismatch: 0x%02X (expected 0x%02X)\n",
               m_instance, who_am_i, ICM42688P_WHO_AM_I_VALUE);
        return -ENODEV;
    }

    syslog(LOG_INFO, "[ICM42688P-%d] WHO_AM_I OK: 0x%02X\n", m_instance, who_am_i);

    /* Select Bank 0 */
    write_reg(ICM42688P_REG_BANK_SEL, 0x00);

    /* Configure gyroscope: ±2000 dps, 1kHz ODR */
    write_reg(ICM42688P_REG_GYRO_CONFIG0, 
              ICM42688P_GYRO_FS_2000DPS | ICM42688P_GYRO_ODR_1KHZ);

    /* Configure accelerometer: ±16g, 1kHz ODR */
    write_reg(ICM42688P_REG_ACCEL_CONFIG0,
              ICM42688P_ACCEL_FS_16G | ICM42688P_ACCEL_ODR_1KHZ);

    /* Enable gyro and accel in low-noise mode */
    write_reg(ICM42688P_REG_PWR_MGMT0,
              ICM42688P_GYRO_MODE_LN | ICM42688P_ACCEL_MODE_LN);

    usleep(1000);  /* Wait 1ms for sensors to stabilize */

    m_initialized = true;
    syslog(LOG_INFO, "[ICM42688P-%d] Initialized successfully\n", m_instance);

    return 0;
}

int ICM42688P::read_raw(struct icm42688p_raw_data_s *data)
{
    uint8_t buffer[15];

    if (!m_initialized)
    {
        return -ENXIO;
    }

    /* Read all sensor data in one burst (temp + accel + gyro = 14 bytes) */
    read_regs(ICM42688P_REG_TEMP_DATA1, buffer, 14);

    /* Parse data (big-endian from sensor) */
    data->temp    = (int16_t)((buffer[0] << 8) | buffer[1]);
    data->accel_x = (int16_t)((buffer[2] << 8) | buffer[3]);
    data->accel_y = (int16_t)((buffer[4] << 8) | buffer[5]);
    data->accel_z = (int16_t)((buffer[6] << 8) | buffer[7]);
    data->gyro_x  = (int16_t)((buffer[8] << 8) | buffer[9]);
    data->gyro_y  = (int16_t)((buffer[10] << 8) | buffer[11]);
    data->gyro_z  = (int16_t)((buffer[12] << 8) | buffer[13]);

    return 0;
}

int ICM42688P::read(struct icm42688p_data_s *data)
{
    struct icm42688p_raw_data_s raw;
    int ret;

    ret = read_raw(&raw);
    if (ret < 0)
    {
        return ret;
    }

    /* Convert to physical units */
    data->accel_x = (float)raw.accel_x * ICM42688P_ACCEL_SCALE_16G;
    data->accel_y = (float)raw.accel_y * ICM42688P_ACCEL_SCALE_16G;
    data->accel_z = (float)raw.accel_z * ICM42688P_ACCEL_SCALE_16G;

    data->gyro_x = (float)raw.gyro_x * ICM42688P_GYRO_SCALE_2000DPS;
    data->gyro_y = (float)raw.gyro_y * ICM42688P_GYRO_SCALE_2000DPS;
    data->gyro_z = (float)raw.gyro_z * ICM42688P_GYRO_SCALE_2000DPS;

    data->temperature = (float)raw.temp * ICM42688P_TEMP_SCALE + ICM42688P_TEMP_OFFSET;

    return 0;
}
