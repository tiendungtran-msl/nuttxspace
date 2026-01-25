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
 *   Low-level SPI burst read với dynamic memory allocation
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

  /* Prepare buffers */
  memset(tx_buffer, 0, length + 1);
  memset(rx_buffer, 0, length + 1);
  tx_buffer[0] = reg | 0x80;  /* Set bit 7 for read */

  /* Configure transaction */
  memset(&trans, 0, sizeof(trans));
  trans.deselect = true;
  trans.nwords = length + 1;
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
              syslog(LOG_WARNING, "[ICM%d] Invalid burst data detected\n", 
                     dev->id);
              ret = ICM42688P_ERR_COMM;
            }
        }

      if (retry < ICM42688P_SPI_RETRY_COUNT - 1)
        {
          syslog(LOG_WARNING, "[ICM%d] SPI burst read retry %d/%d\n",
                 dev->id, retry + 1, ICM42688P_SPI_RETRY_COUNT);
          usleep(ICM42688P_SPI_RETRY_DELAY_US);
        }
    }

  syslog(LOG_ERR, "[ICM%d] SPI burst read failed after %d retries\n",
         dev->id, ICM42688P_SPI_RETRY_COUNT);
  
  dev->error_count++;
  dev->last_error_code = ICM42688P_ERR_COMM;

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

/* Continue to Part 2... */