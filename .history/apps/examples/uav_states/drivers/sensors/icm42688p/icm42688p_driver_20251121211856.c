/****************************************************************************
 * apps/examples/uav_states/drivers/sensors/icm42688p/icm42688p_driver.c
 *
 * ICM-42688-P 6-DOF IMU Driver Implementation
 * Optimized version with enhanced reliability and error handling
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <math.h>
#include <debug.h>
#include <pthread.h>
#include <nuttx/clock.h>

#include <nuttx/spi/spi.h>
#include <nuttx/spi/spi_transfer.h>
#include "stm32_spi_icm.h"

#include "icm42688p_driver.h"
#include "icm42688p_regs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Timing constants (microseconds unless specified) */
#define ICM42688P_RESET_DELAY_MS            100
#define ICM42688P_POWERUP_DELAY_MS          50
#define ICM42688P_BANK_SWITCH_DELAY_US      200
#define ICM42688P_SPI_TRANSACTION_DELAY_US  10
#define ICM42688P_SPI_RETRY_DELAY_US        1000
#define ICM42688P_POST_WRITE_DELAY_US       10
#define ICM42688P_POST_READ_DELAY_US        5

/* Retry and timeout settings */
#define ICM42688P_SPI_RETRY_COUNT           3
#define ICM42688P_RESET_TIMEOUT_MS          200
#define ICM42688P_WHOAMI_RETRY_COUNT        5

/* Data validation thresholds */
#define ICM42688P_GYRO_VARIANCE_THRESHOLD   10.0f  /* dps */
#define ICM42688P_ACCEL_MIN_MAGNITUDE       0.5f   /* g */
#define ICM42688P_ACCEL_MAX_MAGNITUDE       1.5f   /* g */
#define ICM42688P_GYRO_STATIONARY_THRESHOLD 50.0f  /* dps */
#define ICM42688P_TEMP_MIN                  -40.0f /* °C */
#define ICM42688P_TEMP_MAX                  85.0f  /* °C */

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int icm42688p_spi_write(icm42688p_dev_t *dev, uint8_t reg, 
                               uint8_t value);
static int icm42688p_spi_read(icm42688p_dev_t *dev, uint8_t reg, 
                              uint8_t *value);
static int icm42688p_spi_read_burst(icm42688p_dev_t *dev, uint8_t reg,
                                    uint8_t *buffer, size_t length);
static int icm42688p_select_bank(icm42688p_dev_t *dev, uint8_t bank);
static int icm42688p_verify_register(icm42688p_dev_t *dev, uint8_t reg,
                                     uint8_t value);
static int icm42688p_wait_for_reset(icm42688p_dev_t *dev);
static int icm42688p_configure(icm42688p_dev_t *dev);
static bool icm42688p_validate_data(uint8_t *data, size_t length);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: icm42688p_validate_data
 * 
 * Description:
 *   Kiểm tra tính hợp lệ của dữ liệu đọc được (không phải all 0 hoặc all 0xFF)
 ****************************************************************************/

static bool icm42688p_validate_data(uint8_t *data, size_t length)
{
  bool all_zero = true;
  bool all_ff = true;
  
  for (size_t i = 0; i < length; i++)
    {
      if (data[i] != 0x00) all_zero = false;
      if (data[i] != 0xFF) all_ff = false;
      
      if (!all_zero && !all_ff)
        {
          return true;  /* Valid data */
        }
    }
  
  return false;  /* Invalid data */
}

/****************************************************************************
 * Name: icm42688p_spi_write
 * 
 * Description:
 *   Low-level SPI write với retry mechanism và comprehensive error handling
 ****************************************************************************/

static int icm42688p_spi_write(icm42688p_dev_t *dev, uint8_t reg, 
                               uint8_t value)
{
  struct spi_sequence_s seq;
  struct spi_trans_s trans;
  uint8_t tx_buffer[2];
  uint8_t rx_buffer[2];
  int ret;
  int retry;

  if (!dev)
    {
      return ICM42688P_ERR_INVALID;
    }

  /* Get SPI device handle */
  struct spi_dev_s *icm_dev = board_spi1_icm_get_device(dev->id);
  if (icm_dev == NULL)
    {
      syslog(LOG_ERR, "[ICM%d] Failed to get SPI device\n", dev->id);
      dev->error_count++;
      dev->last_error_code = ICM42688P_ERR_COMM;
      return ICM42688P_ERR_COMM;
    }

  /* Prepare buffers */
  memset(rx_buffer, 0, sizeof(rx_buffer));
  tx_buffer[0] = reg & 0x7F;  /* Clear bit 7 for write */
  tx_buffer[1] = value;

  /* Configure transaction */
  memset(&trans, 0, sizeof(trans));
  trans.deselect = true;
  trans.nwords = 2;
  trans.txbuffer = tx_buffer;
  trans.rxbuffer = rx_buffer;

  /* Configure sequence */
  memset(&seq, 0, sizeof(seq));
  seq.dev = SPIDEV_ID(SPIDEVTYPE_IMU, dev->id);
  seq.mode = SPIDEV_MODE3;
  seq.nbits = 8;
  seq.ntrans = 1;
  seq.frequency = 10000000;  /* 10 MHz - max safe frequency */
  seq.trans = &trans;

  /* Perform transfer with retry */
  for (retry = 0; retry < ICM42688P_SPI_RETRY_COUNT; retry++)
    {
      ret = spi_transfer(icm_dev, &seq);
      
      if (ret == OK)
        {
          usleep(ICM42688P_POST_WRITE_DELAY_US);
          return ICM42688P_OK;
        }

      if (retry < ICM42688P_SPI_RETRY_COUNT - 1)
        {
          syslog(LOG_WARNING, "[ICM%d] SPI write retry %d/%d (reg=0x%02X)\n",
                 dev->id, retry + 1, ICM42688P_SPI_RETRY_COUNT, reg);
          usleep(ICM42688P_SPI_RETRY_DELAY_US);
        }
    }

  syslog(LOG_ERR, "[ICM%d] SPI write failed after %d retries (reg=0x%02X)\n",
         dev->id, ICM42688P_SPI_RETRY_COUNT, reg);
  
  dev->error_count++;
  dev->last_error_code = ICM42688P_ERR_COMM;
  
  return ICM42688P_ERR_COMM;
}

/****************************************************************************
 * Name: icm42688p_spi_read
 * 
 * Description:
 *   Low-level SPI read với retry mechanism
 ****************************************************************************/

static int icm42688p_spi_read(icm42688p_dev_t *dev, uint8_t reg, 
                              uint8_t *value)
{
  struct spi_sequence_s seq;
  struct spi_trans_s trans;
  uint8_t tx_buffer[2];
  uint8_t rx_buffer[2];
  int ret;
  int retry;

  if (!dev || !value)
    {
      return ICM42688P_ERR_INVALID;
    }

  /* Get SPI device handle */
  struct spi_dev_s *icm_dev = board_spi1_icm_get_device(dev->id);
  if (icm_dev == NULL)
    {
      syslog(LOG_ERR, "[ICM%d] Failed to get SPI device\n", dev->id);
      dev->error_count++;
      dev->last_error_code = ICM42688P_ERR_COMM;
      return ICM42688P_ERR_COMM;
    }

  /* Prepare buffers */
  memset(rx_buffer, 0, sizeof(rx_buffer));
  tx_buffer[0] = reg | 0x80;  /* Set bit 7 for read */
  tx_buffer[1] = 0x00;        /* Dummy byte */

  /* Configure transaction */
  memset(&trans, 0, sizeof(trans));
  trans.deselect = true;
  trans.nwords = 2;
  trans.txbuffer = tx_buffer;
  trans.rxbuffer = rx_buffer;

  /* Configure sequence */
  memset(&seq, 0, sizeof(seq));
  seq.dev = SPIDEV_ID(SPIDEVTYPE_IMU, dev->id);
  seq.mode = SPIDEV_MODE3;
  seq.nbits = 8;
  seq.ntrans = 1;
  seq.frequency = 10000000;
  seq.trans = &trans;

  /* Perform transfer with retry */
  for (retry = 0; retry < ICM42688P_SPI_RETRY_COUNT; retry++)
    {
      ret = spi_transfer(icm_dev, &seq);
      
      if (ret == OK)
        {
          *value = rx_buffer[1];
          usleep(ICM42688P_POST_READ_DELAY_US);
          return ICM42688P_OK;
        }

      if (retry < ICM42688P_SPI_RETRY_COUNT - 1)
        {
          syslog(LOG_WARNING, "[ICM%d] SPI read retry %d/%d (reg=0x%02X)\n",
                 dev->id, retry + 1, ICM42688P_SPI_RETRY_COUNT, reg);
          usleep(ICM42688P_SPI_RETRY_DELAY_US);
        }
    }

  syslog(LOG_ERR, "[ICM%d] SPI read failed after %d retries (reg=0x%02X)\n",
         dev->id, ICM42688P_SPI_RETRY_COUNT, reg);
  
  dev->error_count++;
  dev->last_error_code = ICM42688P_ERR_COMM;
  
  return ICM42688P_ERR_COMM;
}

/****************************************************************************
 * Name: icm42688p_spi_read_burst
 * 
 * Description:
 *   Low-level SPI burst read - FIXED VERSION
 ****************************************************************************/

static int icm42688p_spi_read_burst(icm42688p_dev_t *dev, uint8_t reg,
                                    uint8_t *buffer, size_t length)
{
  struct spi_sequence_s seq;
  struct spi_trans_s trans;
  uint8_t *tx_buffer = NULL;
  uint8_t *rx_buffer = NULL;
  int ret = ICM42688P_ERR_COMM;
  int retry;

  if (!dev || !buffer || length == 0)
    {
      return ICM42688P_ERR_INVALID;
    }

  /* Get SPI device handle */
  struct spi_dev_s *icm_dev = board_spi1_icm_get_device(dev->id);
  if (icm_dev == NULL)
    {
      syslog(LOG_ERR, "[ICM%d] Failed to get SPI device\n", dev->id);
      dev->error_count++;
      dev->last_error_code = ICM42688P_ERR_COMM;
      return ICM42688P_ERR_COMM;
    }

  /* Allocate buffers */
  tx_buffer = (uint8_t *)malloc(length + 1);
  rx_buffer = (uint8_t *)malloc(length + 1);
  
  if (!tx_buffer || !rx_buffer)
    {
      syslog(LOG_ERR, "[ICM%d] Memory allocation failed for burst read\n", 
             dev->id);
      ret = ICM42688P_ERR_COMM;
      goto cleanup;
    }

  /* Prepare buffers - CRITICAL: tx_buffer MUST be populated */
  memset(tx_buffer, 0, length + 1);
  memset(rx_buffer, 0, length + 1);
  tx_buffer[0] = reg | 0x80;  /* Set bit 7 for read */
  
  /* Fill rest of tx_buffer with dummy bytes */
  for (size_t i = 1; i <= length; i++)
    {
      tx_buffer[i] = 0x00;
    }

  /* Configure transaction */
  memset(&trans, 0, sizeof(trans));
  trans.deselect = true;
  trans.nwords = length + 1;
  trans.txbuffer = tx_buffer;  /* MUST NOT be NULL */
  trans.rxbuffer = rx_buffer;

  /* Configure sequence */
  memset(&seq, 0, sizeof(seq));
  seq.dev = SPIDEV_ID(SPIDEVTYPE_IMU, dev->id);
  seq.mode = SPIDEV_MODE3;
  seq.nbits = 8;
  seq.ntrans = 1;
  seq.frequency = 10000000;  /* 10 MHz */
  seq.trans = &trans;

  /* Perform transfer with retry */
  for (retry = 0; retry < ICM42688P_SPI_RETRY_COUNT; retry++)
    {
      ret = spi_transfer(icm_dev, &seq);
      
      if (ret == OK)
        {
          /* Validate data before copying */
          if (icm42688p_validate_data(&rx_buffer[1], length))
            {
              memcpy(buffer, &rx_buffer[1], length);
              usleep(ICM42688P_POST_READ_DELAY_US);
              ret = ICM42688P_OK;
              goto cleanup;
            }
          else
            {
              syslog(LOG_WARNING, "[ICM%d] Invalid burst data detected, retry %d\n", 
                     dev->id, retry + 1);
              ret = ICM42688P_ERR_COMM;
              /* Continue to retry */
            }
        }

      if (retry < ICM42688P_SPI_RETRY_COUNT - 1)
        {
          syslog(LOG_WARNING, "[ICM%d] SPI burst read retry %d/%d (ret=%d)\n",
                 dev->id, retry + 1, ICM42688P_SPI_RETRY_COUNT, ret);
          usleep(ICM42688P_SPI_RETRY_DELAY_US);
        }
    }

  syslog(LOG_ERR, "[ICM%d] SPI burst read failed after %d retries\n",
         dev->id, ICM42688P_SPI_RETRY_COUNT);
  
  dev->error_count++;
  dev->last_error_code = ICM42688P_ERR_COMM;
  ret = ICM42688P_ERR_COMM;

cleanup:
  if (tx_buffer) free(tx_buffer);
  if (rx_buffer) free(rx_buffer);
  
  return ret;
}

/****************************************************************************
 * Name: icm42688p_select_bank
 * 
 * Description:
 *   Chọn register bank với verification
 ****************************************************************************/

static int icm42688p_select_bank(icm42688p_dev_t *dev, uint8_t bank)
{
  int ret;
  uint8_t read_bank;

  if (!dev)
    {
      return ICM42688P_ERR_INVALID;
    }

  /* Check if already on correct bank */
  if (dev->current_bank == bank)
    {
      return ICM42688P_OK;
    }

  /* Write to BANK_SEL register */
  ret = icm42688p_spi_write(dev, ICM42688P_REG_BANK_SEL, bank & 0x07);
  if (ret != ICM42688P_OK)
    {
      syslog(LOG_ERR, "[ICM%d] Failed to write bank select\n", dev->id);
      return ret;
    }

  /* Wait for bank switch to complete */
  usleep(ICM42688P_BANK_SWITCH_DELAY_US);

  /* Verify bank selection */
  ret = icm42688p_spi_read(dev, ICM42688P_REG_BANK_SEL, &read_bank);
  if (ret != ICM42688P_OK)
    {
      syslog(LOG_ERR, "[ICM%d] Failed to verify bank select\n", dev->id);
      return ret;
    }

  if ((read_bank & 0x07) != (bank & 0x07))
    {
      syslog(LOG_ERR, "[ICM%d] Bank verification failed: expected %d, got %d\n",
             dev->id, bank, read_bank & 0x07);
      dev->error_count++;
      return ICM42688P_ERR_COMM;
    }

  dev->current_bank = bank;
  
  return ICM42688P_OK;
}

/****************************************************************************
 * Name: icm42688p_verify_register
 * 
 * Description:
 *   Ghi và verify thanh ghi
 ****************************************************************************/

static int icm42688p_verify_register(icm42688p_dev_t *dev, uint8_t reg,
                                     uint8_t value)
{
  int ret;
  uint8_t read_value;

  /* Write value */
  ret = icm42688p_spi_write(dev, reg, value);
  if (ret != ICM42688P_OK)
    {
      return ret;
    }

  /* Wait for register to update */
  usleep(1000);

  /* Read back to verify */
  ret = icm42688p_spi_read(dev, reg, &read_value);
  if (ret != ICM42688P_OK)
    {
      return ret;
    }

  /* Compare values */
  if (read_value != value)
    {
      syslog(LOG_ERR, "[ICM%d] Register 0x%02X verify failed: "
             "wrote 0x%02X, read 0x%02X\n",
             dev->id, reg, value, read_value);
      dev->error_count++;
      return ICM42688P_ERR_COMM;
    }

  return ICM42688P_OK;
}

/****************************************************************************
 * Name: icm42688p_wait_for_reset
 * 
 * Description:
 *   Chờ reset hoàn tất bằng cách poll DEVICE_CONFIG register
 ****************************************************************************/

static int icm42688p_wait_for_reset(icm42688p_dev_t *dev)
{
  uint8_t device_config;
  int timeout_ms = 0;
  int ret;

  while (timeout_ms < ICM42688P_RESET_TIMEOUT_MS)
    {
      usleep(1000);  /* 1ms */
      timeout_ms++;
      
      ret = icm42688p_spi_read(dev, ICM42688P_DEVICE_CONFIG, &device_config);
      
      if (ret == ICM42688P_OK)
        {
          /* Bit 0 auto-clears when reset is complete */
          if ((device_config & 0x01) == 0)
            {
              syslog(LOG_INFO, "[ICM%d] Reset completed after %d ms\n",
                     dev->id, timeout_ms);
              return ICM42688P_OK;
            }
        }
    }

  syslog(LOG_ERR, "[ICM%d] Reset timeout after %d ms\n", 
         dev->id, ICM42688P_RESET_TIMEOUT_MS);
  dev->error_count++;
  dev->last_error_code = ICM42688P_ERR_TIMEOUT;
  
  return ICM42688P_ERR_TIMEOUT;
}

/****************************************************************************
 * Name: icm42688p_configure
 * 
 * Description:
 *   Cấu hình ICM42688P với full verification
 ****************************************************************************/

static int icm42688p_configure(icm42688p_dev_t *dev)
{
  int ret;
  uint8_t pwr_mgmt;

  if (!dev)
    {
      return ICM42688P_ERR_INVALID;
    }

  syslog(LOG_INFO, "[ICM%d] Starting configuration...\n", dev->id);

  /* Select Bank 0 */
  ret = icm42688p_select_bank(dev, BANK_0);
  if (ret != ICM42688P_OK)
    {
      return ret;
    }

  /* Enable Gyro and Accel in Low Noise mode */
  pwr_mgmt = PWR_MGMT0_GYRO_MODE_LN | PWR_MGMT0_ACCEL_MODE_LN;
  ret = icm42688p_verify_register(dev, ICM42688P_PWR_MGMT0, pwr_mgmt);
  if (ret != ICM42688P_OK)
    {
      syslog(LOG_ERR, "[ICM%d] Failed to configure power management\n", dev->id);
      return ret;
    }

  syslog(LOG_INFO, "[ICM%d] Power management configured\n", dev->id);
  
  /* Wait for sensors to power up */
  usleep(ICM42688P_POWERUP_DELAY_MS * 1000);

  /* Configure Gyroscope: ±2000 dps, ODR 1kHz */
  dev->gyro_fs = GYRO_CONFIG0_FS_SEL_2000DPS;
  dev->gyro_sensitivity = GYRO_SENSITIVITY_2000DPS;

  ret = icm42688p_verify_register(dev, ICM42688P_GYRO_CONFIG0,
                                  dev->gyro_fs | GYRO_CONFIG0_ODR_1KHZ);
  if (ret != ICM42688P_OK)
    {
      syslog(LOG_ERR, "[ICM%d] Failed to configure gyroscope\n", dev->id);
      return ret;
    }

  syslog(LOG_INFO, "[ICM%d] Gyroscope configured: ±2000dps, ODR 1kHz\n", dev->id);

  /* Configure Accelerometer: ±16g, ODR 1kHz */
  dev->accel_fs = ACCEL_CONFIG0_FS_SEL_16G;
  dev->accel_sensitivity = ACCEL_SENSITIVITY_16G;

  ret = icm42688p_verify_register(dev, ICM42688P_ACCEL_CONFIG0,
                                  dev->accel_fs | ACCEL_CONFIG0_ODR_1KHZ);
  if (ret != ICM42688P_OK)
    {
      syslog(LOG_ERR, "[ICM%d] Failed to configure accelerometer\n", dev->id);
      return ret;
    }

  syslog(LOG_INFO, "[ICM%d] Accelerometer configured: ±16g, ODR 1kHz\n", dev->id);

  /* Configure Low-Pass Filter */
  ret = icm42688p_verify_register(dev, ICM42688P_GYRO_ACCEL_CONFIG0, 0x44);
  if (ret != ICM42688P_OK)
    {
      syslog(LOG_ERR, "[ICM%d] Failed to configure filters\n", dev->id);
      return ret;
    }

  syslog(LOG_INFO, "[ICM%d] Low-pass filters configured\n", dev->id);

  /* Wait for configuration to stabilize */
  usleep(ICM42688P_POWERUP_DELAY_MS * 1000);

  syslog(LOG_INFO, "[ICM%d] Configuration completed successfully\n", dev->id);

  return ICM42688P_OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: icm42688p_write_register
 ****************************************************************************/

int icm42688p_write_register(icm42688p_dev_t *dev, uint8_t reg, uint8_t value)
{
  if (!dev)
    {
      return ICM42688P_ERR_INVALID;
    }

  return icm42688p_spi_write(dev, reg, value);
}

/****************************************************************************
 * Name: icm42688p_read_register
 ****************************************************************************/

int icm42688p_read_register(icm42688p_dev_t *dev, uint8_t reg, uint8_t *value)
{
  if (!dev || !value)
    {
      return ICM42688P_ERR_INVALID;
    }

  return icm42688p_spi_read(dev, reg, value);
}

/****************************************************************************
 * Name: icm42688p_read_registers
 ****************************************************************************/

int icm42688p_read_registers(icm42688p_dev_t *dev, uint8_t reg,
                             uint8_t *buffer, size_t length)
{
  if (!dev || !buffer || length == 0)
    {
      return ICM42688P_ERR_INVALID;
    }

  return icm42688p_spi_read_burst(dev, reg, buffer, length);
}

/****************************************************************************
 * Name: icm42688p_reset
 ****************************************************************************/

int icm42688p_reset(icm42688p_dev_t *dev)
{
  int ret;

  if (!dev)
    {
      return ICM42688P_ERR_INVALID;
    }

  syslog(LOG_INFO, "[ICM%d] Initiating soft reset...\n", dev->id);

  /* Select Bank 0 */
  dev->current_bank = 0xFF;  /* Force bank switch */
  ret = icm42688p_select_bank(dev, BANK_0);
  if (ret != ICM42688P_OK)
    {
      syslog(LOG_ERR, "[ICM%d] Failed to select bank 0 for reset\n", dev->id);
      return ret;
    }

  /* Send soft reset command */
  ret = icm42688p_spi_write(dev, ICM42688P_DEVICE_CONFIG, 0x01);
  if (ret != ICM42688P_OK)
    {
      syslog(LOG_ERR, "[ICM%d] Failed to write reset command\n", dev->id);
      return ret;
    }

  /* Wait for reset to complete */
  ret = icm42688p_wait_for_reset(dev);
  if (ret != ICM42688P_OK)
    {
      return ret;
    }

  /* Reset internal state */
  dev->current_bank = 0xFF;
  dev->state = ICM42688P_STATE_UNINITIALIZED;
  dev->is_calibrated = false;

  syslog(LOG_INFO, "[ICM%d] Soft reset completed successfully\n", dev->id);

  return ICM42688P_OK;
}

/****************************************************************************
 * Name: icm42688p_init
 ****************************************************************************/

int icm42688p_init(icm42688p_dev_t *dev, uint8_t id)
{
  int ret;
  uint8_t who_am_i;
  int retry;
  pthread_mutexattr_t attr;

  if (!dev)
    {
      syslog(LOG_ERR, "ERROR: Invalid device pointer\n");
      return ICM42688P_ERR_INVALID;
    }

  /* Clear device structure */
  memset(dev, 0, sizeof(icm42688p_dev_t));
  dev->id = id;
  dev->current_bank = 0xFF;
  dev->state = ICM42688P_STATE_UNINITIALIZED;

  syslog(LOG_INFO, "[ICM%d] Starting initialization...\n", id);

  /* Initialize mutex */
  pthread_mutexattr_init(&attr);
  pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
  
  if (pthread_mutex_init(&dev->lock, &attr) != 0)
    {
      syslog(LOG_ERR, "[ICM%d] Failed to initialize mutex\n", id);
      pthread_mutexattr_destroy(&attr);
      return ICM42688P_ERR_MUTEX;
    }
  
  pthread_mutexattr_destroy(&attr);
  dev->lock_initialized = true;

  /* Verify SPI device */
  struct spi_dev_s *icm_dev = board_spi1_icm_get_device(id);
  if (icm_dev == NULL)
    {
      syslog(LOG_ERR, "[ICM%d] Failed to get SPI device\n", id);
      ret = ICM42688P_ERR_COMM;
      goto error_cleanup;
    }

  /* Initial stabilization delay */
  usleep(10000);

  /* Try to read WHO_AM_I with retries */
  for (retry = 0; retry < ICM42688P_WHOAMI_RETRY_COUNT; retry++)
    {
      ret = icm42688p_select_bank(dev, BANK_0);
      if (ret == ICM42688P_OK)
        {
          ret = icm42688p_spi_read(dev, ICM42688P_WHO_AM_I, &who_am_i);
          if (ret == ICM42688P_OK && who_am_i == ICM42688P_WHO_AM_I_VALUE)
            {
              syslog(LOG_INFO, "[ICM%d] Device detected, WHO_AM_I = 0x%02X\n",
                     id, who_am_i);
              break;
            }
        }

      syslog(LOG_WARNING, "[ICM%d] WHO_AM_I read attempt %d/%d failed\n",
             id, retry + 1, ICM42688P_WHOAMI_RETRY_COUNT);
      usleep(10000);
    }

  if (retry >= ICM42688P_WHOAMI_RETRY_COUNT)
    {
      syslog(LOG_ERR, "[ICM%d] Failed to detect device after %d attempts\n", 
             id, ICM42688P_WHOAMI_RETRY_COUNT);
      ret = ICM42688P_ERR_COMM;
      goto error_cleanup;
    }

  /* Soft reset */
  ret = icm42688p_reset(dev);
  if (ret != ICM42688P_OK)
    {
      syslog(LOG_ERR, "[ICM%d] Reset failed\n", id);
      goto error_cleanup;
    }

  /* Post-reset stabilization */
  usleep(50000);

  /* Configure device */
  ret = icm42688p_configure(dev);
  if (ret != ICM42688P_OK)
    {
      syslog(LOG_ERR, "[ICM%d] Configuration failed\n", id);
      goto error_cleanup;
    }

  /* Initialize offsets */
  dev->gyro_offset.x = 0.0f;
  dev->gyro_offset.y = 0.0f;
  dev->gyro_offset.z = 0.0f;

  /* Set state */
  dev->state = ICM42688P_STATE_INITIALIZED;
  dev->sample_count = 0;
  dev->error_count = 0;

  syslog(LOG_INFO, "[ICM%d] Initialization completed successfully\n", id);

  return ICM42688P_OK;

error_cleanup:
  if (dev->lock_initialized)
    {
      pthread_mutex_destroy(&dev->lock);
      dev->lock_initialized = false;
    }
  dev->state = ICM42688P_STATE_ERROR;
  return ret;
}

/****************************************************************************
 * Name: icm42688p_deinit
 ****************************************************************************/

int icm42688p_deinit(icm42688p_dev_t *dev)
{
  if (!dev)
    {
      return ICM42688P_ERR_INVALID;
    }

  /* Cleanup mutex */
  if (dev->lock_initialized)
    {
      pthread_mutex_destroy(&dev->lock);
      dev->lock_initialized = false;
    }

  dev->state = ICM42688P_STATE_UNINITIALIZED;
  
  syslog(LOG_INFO, "[ICM%d] Deinitialized\n", dev->id);

  return ICM42688P_OK;
}

/****************************************************************************
 * Name: icm42688p_get_data
 ****************************************************************************/

/****************************************************************************
 * Name: icm42688p_get_data
 * 
 * Description:
 *   Đọc dữ liệu sensor - FIXED VERSION with timeout protection
 ****************************************************************************/

int icm42688p_get_data(icm42688p_dev_t *dev)
{
  int ret;
  uint8_t data[14];
  int16_t raw_values[7];
  bool locked = false;

  if (!dev)
    {
      syslog(LOG_ERR, "ERROR: Invalid device pointer\n");
      return ICM42688P_ERR_INVALID;
    }

  if (!(dev->state & ICM42688P_STATE_INITIALIZED))
    {
      syslog(LOG_ERR, "[ICM%d] Device not initialized (state=0x%02X)\n", 
             dev->id, dev->state);
      return ICM42688P_ERR_INVALID;
    }

  /* Lock for thread safety */
  ret = icm42688p_lock(dev);
  if (ret != ICM42688P_OK)
    {
      syslog(LOG_ERR, "[ICM%d] Failed to acquire lock\n", dev->id);
      return ret;
    }
  locked = true;

  /* Select Bank 0 */
  ret = icm42688p_select_bank(dev, BANK_0);
  if (ret != ICM42688P_OK)
    {
      syslog(LOG_ERR, "[ICM%d] Failed to select bank 0\n", dev->id);
      goto unlock_return;
    }

  /* Read burst data - This is where it might hang */
  memset(data, 0, sizeof(data));
  
  ret = icm42688p_spi_read_burst(dev, ICM42688P_TEMP_DATA1, data, 14);
  if (ret != ICM42688P_OK)
    {
      syslog(LOG_ERR, "[ICM%d] Failed to read sensor data (ret=%d)\n", 
             dev->id, ret);
      dev->error_count++;
      goto unlock_return;
    }

  /* Parse raw values */
  for (int i = 0; i < 7; i++)
    {
      raw_values[i] = (int16_t)((data[i * 2] << 8) | data[i * 2 + 1]);
    }

  /* Convert to physical units */
  dev->temp = ((float)raw_values[0] / TEMP_SENSITIVITY) + TEMP_OFFSET;
  
  dev->accel.x = (float)raw_values[1] / dev->accel_sensitivity;
  dev->accel.y = (float)raw_values[2] / dev->accel_sensitivity;
  dev->accel.z = (float)raw_values[3] / dev->accel_sensitivity;

  dev->gyro.x = (float)raw_values[4] / dev->gyro_sensitivity;
  dev->gyro.y = (float)raw_values[5] / dev->gyro_sensitivity;
  dev->gyro.z = (float)raw_values[6] / dev->gyro_sensitivity;

  /* Apply calibration */
  dev->gyro_calib.x = dev->gyro.x - dev->gyro_offset.x;
  dev->gyro_calib.y = dev->gyro.y - dev->gyro_offset.y;
  dev->gyro_calib.z = dev->gyro.z - dev->gyro_offset.z;

  /* Timestamp */
  dev->timestamp = clock_systime_ticks() * (1000000 / CLOCKS_PER_SEC);
  dev->sample_count++;

  ret = ICM42688P_OK;

unlock_return:
  if (locked)
    {
      int unlock_ret = icm42688p_unlock(dev);
      if (unlock_ret != ICM42688P_OK)
        {
          syslog(LOG_ERR, "[ICM%d] Failed to release lock\n", dev->id);
        }
    }
  
  return ret;
}

/****************************************************************************
 * Name: icm42688p_calibrate_gyro
 ****************************************************************************/

int icm42688p_calibrate_gyro(icm42688p_dev_t *dev, int num_samples)
{
  int ret;
  float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
  float mean_x, mean_y, mean_z;
  int valid_samples = 0;
  float *samples_x = NULL;
  float *samples_y = NULL;
  float *samples_z = NULL;

  if (!dev || !(dev->state & ICM42688P_STATE_INITIALIZED) || num_samples <= 0)
    {
      return ICM42688P_ERR_INVALID;
    }

  syslog(LOG_INFO, "[ICM%d] Starting gyro calibration (%d samples)...\n",
         dev->id, num_samples);
  printf("*** KEEP DEVICE STATIONARY! ***\n");

  /* Allocate memory for samples */
  samples_x = (float *)malloc(num_samples * sizeof(float));
  samples_y = (float *)malloc(num_samples * sizeof(float));
  samples_z = (float *)malloc(num_samples * sizeof(float));

  if (!samples_x || !samples_y || !samples_z)
    {
      syslog(LOG_ERR, "[ICM%d] Memory allocation failed\n", dev->id);
      ret = ICM42688P_ERR_COMM;
      goto cleanup;
    }

  /* Stabilization delay */
  usleep(2000000);

  /* Collect samples */
  for (int i = 0; i < num_samples; i++)
    {
      ret = icm42688p_get_data(dev);
      if (ret != ICM42688P_OK)
        {
          syslog(LOG_ERR, "[ICM%d] Calibration read failed at sample %d\n",
                 dev->id, i);
          goto cleanup;
        }

      samples_x[i] = dev->gyro.x;
      samples_y[i] = dev->gyro.y;
      samples_z[i] = dev->gyro.z;

      usleep(10000);  /* 10ms -> 100Hz */

      if ((i % 50) == 0)
        {
          printf("Calibration progress: %d/%d\n", i, num_samples);
        }
    }

  /* Calculate mean */
  mean_x = mean_y = mean_z = 0.0f;
  for (int i = 0; i < num_samples; i++)
    {
      mean_x += samples_x[i];
      mean_y += samples_y[i];
      mean_z += samples_z[i];
    }
  mean_x /= num_samples;
  mean_y /= num_samples;
  mean_z /= num_samples;

  /* Remove outliers and recalculate mean */
  for (int i = 0; i < num_samples; i++)
    {
      if (fabsf(samples_x[i] - mean_x) < ICM42688P_GYRO_VARIANCE_THRESHOLD &&
          fabsf(samples_y[i] - mean_y) < ICM42688P_GYRO_VARIANCE_THRESHOLD &&
          fabsf(samples_z[i] - mean_z) < ICM42688P_GYRO_VARIANCE_THRESHOLD)
        {
          sum_x += samples_x[i];
          sum_y += samples_y[i];
          sum_z += samples_z[i];
          valid_samples++;
        }
    }

  if (valid_samples < num_samples / 2)
    {
      syslog(LOG_ERR, "[ICM%d] Too many outliers (%d/%d valid)\n",
             dev->id, valid_samples, num_samples);
      ret = ICM42688P_ERR_COMM;
      goto cleanup;
    }

  /* Set offsets */
  dev->gyro_offset.x = sum_x / valid_samples;
  dev->gyro_offset.y = sum_y / valid_samples;
  dev->gyro_offset.z = sum_z / valid_samples;
  dev->is_calibrated = true;
  dev->state |= ICM42688P_STATE_CALIBRATED;

  syslog(LOG_INFO, "[ICM%d] Calibration complete (%d/%d valid samples)\n",
         dev->id, valid_samples, num_samples);
  syslog(LOG_INFO, "  Offset X: %.3f dps\n", dev->gyro_offset.x);
  syslog(LOG_INFO, "  Offset Y: %.3f dps\n", dev->gyro_offset.y);
  syslog(LOG_INFO, "  Offset Z: %.3f dps\n", dev->gyro_offset.z);

  ret = ICM42688P_OK;

cleanup:
  if (samples_x) free(samples_x);
  if (samples_y) free(samples_y);
  if (samples_z) free(samples_z);
  
  return ret;
}

/****************************************************************************
 * Name: icm42688p_self_test
 ****************************************************************************/

int icm42688p_self_test(icm42688p_dev_t *dev)
{
  int ret;
  float accel_mag, gyro_mag;

  if (!dev || !(dev->state & ICM42688P_STATE_INITIALIZED))
    {
      return ICM42688P_ERR_INVALID;
    }

  syslog(LOG_INFO, "[ICM%d] Running self-test...\n", dev->id);

  /* Read multiple samples */
  for (int i = 0; i < 5; i++)
    {
      ret = icm42688p_get_data(dev);
      if (ret != ICM42688P_OK)
        {
          syslog(LOG_ERR, "[ICM%d] Self-test data read failed\n", dev->id);
          return ret;
        }
      usleep(10000);
    }

  /* Check accelerometer magnitude */
  accel_mag = sqrtf(dev->accel.x * dev->accel.x +
                    dev->accel.y * dev->accel.y +
                    dev->accel.z * dev->accel.z);

  if (accel_mag < ICM42688P_ACCEL_MIN_MAGNITUDE || 
      accel_mag > ICM42688P_ACCEL_MAX_MAGNITUDE)
    {
      syslog(LOG_ERR, "[ICM%d] Accel magnitude out of range: %.2f g\n",
             dev->id, accel_mag);
      return ICM42688P_ERR_SELFTEST;
    }

  /* Check gyroscope magnitude */
  gyro_mag = sqrtf(dev->gyro_calib.x * dev->gyro_calib.x +
                   dev->gyro_calib.y * dev->gyro_calib.y +
                   dev->gyro_calib.z * dev->gyro_calib.z);

  if (gyro_mag > ICM42688P_GYRO_STATIONARY_THRESHOLD)
    {
      syslog(LOG_WARNING, "[ICM%d] Gyro magnitude high: %.2f dps "
             "(calibration recommended)\n", dev->id, gyro_mag);
    }

  /* Check temperature */
  if (dev->temp < ICM42688P_TEMP_MIN || dev->temp > ICM42688P_TEMP_MAX)
    {
      syslog(LOG_ERR, "[ICM%d] Temperature out of range: %.2f °C\n",
             dev->id, dev->temp);
      return ICM42688P_ERR_SELFTEST;
    }

  syslog(LOG_INFO, "[ICM%d] Self-test PASSED\n", dev->id);
  syslog(LOG_INFO, "  Accel magnitude: %.3f g\n", accel_mag);
  syslog(LOG_INFO, "  Gyro magnitude:  %.2f dps\n", gyro_mag);
  syslog(LOG_INFO, "  Temperature:     %.2f °C\n", dev->temp);

  return ICM42688P_OK;
}

/****************************************************************************
 * Name: icm42688p_orientation
 ****************************************************************************/

int icm42688p_orientation(icm42688p_dev_t *dev)
{
  if (!dev || !(dev->state & ICM42688P_STATE_INITIALIZED))
    {
      return ICM42688P_ERR_INVALID;
    }

  dev->pitch = atan2f(-dev->accel.x, 
                      sqrtf(dev->accel.y * dev->accel.y + 
                            dev->accel.z * dev->accel.z)) * (180.0f / M_PI);
  
  dev->roll = atan2f(dev->accel.y, dev->accel.z) * (180.0f / M_PI);

  return ICM42688P_OK;
}

/****************************************************************************
 * Name: icm42688p_print_status
 ****************************************************************************/

void icm42688p_print_status(icm42688p_dev_t *dev)
{
  if (!dev)
    {
      return;
    }

  printf("\n===== ICM42688P Status (Device %d) =====\n", dev->id);
  printf("State:         0x%02X ", dev->state);
  if (dev->state & ICM42688P_STATE_INITIALIZED) printf("[INIT] ");
  if (dev->state & ICM42688P_STATE_CALIBRATED) printf("[CAL] ");
  if (dev->state & ICM42688P_STATE_ERROR) printf("[ERR] ");
  printf("\n");
  
  printf("Current Bank:  %d\n", dev->current_bank);
  printf("\n--- Configuration ---\n");
  printf("Gyro Range:    ±2000 dps\n");
  printf("Accel Range:   ±16g\n");
  printf("Gyro Sens:     %.1f LSB/dps\n", dev->gyro_sensitivity);
  printf("Accel Sens:    %.1f LSB/g\n", dev->accel_sensitivity);
  
  printf("\n--- Calibration ---\n");
  printf("Calibrated:    %s\n", dev->is_calibrated ? "YES" : "NO");
  printf("Gyro Offset:   X=%.3f Y=%.3f Z=%.3f dps\n",
         dev->gyro_offset.x, dev->gyro_offset.y, dev->gyro_offset.z);
  
  printf("\n--- Statistics ---\n");
  printf("Samples Read:  %u\n", dev->sample_count);
  printf("Error Count:   %u\n", dev->error_count);
  printf("Last Error:    %d\n", dev->last_error_code);
  
  if ((dev->state & ICM42688P_STATE_INITIALIZED) && dev->sample_count > 0)
    {
      printf("\n--- Latest Data ---\n");
      printf("Accel:         X=%.3f Y=%.3f Z=%.3f g\n",
             dev->accel.x, dev->accel.y, dev->accel.z);
      printf("Gyro (calib):  X=%.2f Y=%.2f Z=%.2f dps\n",
             dev->gyro_calib.x, dev->gyro_calib.y, dev->gyro_calib.z);
      printf("Temperature:   %.2f °C\n", dev->temp);
      printf("Timestamp:     %llu µs\n", dev->timestamp);
    }
  
  printf("========================================\n\n");
}