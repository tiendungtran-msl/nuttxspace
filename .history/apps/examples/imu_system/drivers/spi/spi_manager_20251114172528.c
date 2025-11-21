/****************************************************************************
 * apps/examples/imu_system/drivers/spi/spi_manager.c
 *
 * SPI Manager with mutex protection
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <pthread.h>
#include <sys/ioctl.h>

#include <nuttx/spi/spi_transfer.h>

#include "spi_manager.h"
#include "../../utils/config.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IMU_SPI_FREQUENCY  10000000  /* 10 MHz */

/****************************************************************************
 * Private Data
 ****************************************************************************/

static int g_spi_fd = -1;
static pthread_mutex_t g_spi_mutex = PTHREAD_MUTEX_INITIALIZER;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spi_manager_init
 ****************************************************************************/

int spi_manager_init(int bus)
{
  char devpath[16];
  uint32_t mode = SPIDEV_MODE3;
  uint32_t freq = IMU_SPI_FREQUENCY;
  int ret;

  snprintf(devpath, sizeof(devpath), "/dev/spi%d", bus);

  g_spi_fd = open(devpath, O_RDWR);
  if (g_spi_fd < 0)
    {
      syslog(LOG_ERR, "SPI: Failed to open %s: %d\n", devpath, errno);
      return -errno;
    }

  /* Set SPI mode (CPOL=1, CPHA=1) */

  ret = ioctl(g_spi_fd, SPI_IOC_WR_MODE, &mode);
  if (ret < 0)
    {
      syslog(LOG_ERR, "SPI: Failed to set mode: %d\n", errno);
      close(g_spi_fd);
      g_spi_fd = -1;
      return -errno;
    }

  /* Set SPI frequency */

  ret = ioctl(g_spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &freq);
  if (ret < 0)
    {
      syslog(LOG_ERR, "SPI: Failed to set frequency: %d\n", errno);
      close(g_spi_fd);
      g_spi_fd = -1;
      return -errno;
    }

  syslog(LOG_INFO, "SPI: Initialized %s @ %lu Hz\n", 
         devpath, (unsigned long)freq);

  return OK;
}

/****************************************************************************
 * Name: spi_manager_deinit
 ****************************************************************************/

void spi_manager_deinit(void)
{
  if (g_spi_fd >= 0)
    {
      close(g_spi_fd);
      g_spi_fd = -1;
    }
}

/****************************************************************************
 * Name: spi_manager_transfer
 ****************************************************************************/

int spi_manager_transfer(int devid, const uint8_t *txbuf, 
                         uint8_t *rxbuf, size_t len)
{
  struct spi_trans_s trans;
  int ret;

  if (g_spi_fd < 0 || (!txbuf && !rxbuf) || len == 0)
    {
      return -EINVAL;
    }

  pthread_mutex_lock(&g_spi_mutex);

  memset(&trans, 0, sizeof(trans));
  trans.deselect = true;
  trans.devid = devid;
  trans.cmd = 0;
  trans.addr = 0;
  trans.nwords = len;

  if (txbuf)
    {
      trans.txbuffer = (void *)txbuf;
    }

  if (rxbuf)
    {
      trans.rxbuffer = rxbuf;
    }

  ret = ioctl(g_spi_fd, SPIIOC_TRANSFER, (unsigned long)&trans);

  pthread_mutex_unlock(&g_spi_mutex);

  return (ret < 0) ? -errno : OK;
}

/****************************************************************************
 * Name: spi_manager_read_reg
 ****************************************************************************/

int spi_manager_read_reg(int devid, uint8_t reg, uint8_t *value)
{
  uint8_t txbuf[2];
  uint8_t rxbuf[2];
  int ret;

  if (!value)
    {
      return -EINVAL;
    }

  txbuf[0] = reg | 0x80;  /* Read bit */
  txbuf[1] = 0xFF;

  ret = spi_manager_transfer(devid, txbuf, rxbuf, 2);
  if (ret == OK)
    {
      *value = rxbuf[1];
    }

  return ret;
}

/****************************************************************************
 * Name: spi_manager_read_regs
 ****************************************************************************/

int spi_manager_read_regs(int devid, uint8_t reg, 
                          uint8_t *buffer, size_t len)
{
  uint8_t *txbuf;
  uint8_t *rxbuf;
  int ret;
  size_t i;

  if (!buffer || len == 0)
    {
      return -EINVAL;
    }

  txbuf = malloc(len + 1);
  rxbuf = malloc(len + 1);
  if (!txbuf || !rxbuf)
    {
      free(txbuf);
      free(rxbuf);
      return -ENOMEM;
    }

  txbuf[0] = reg | 0x80;  /* Read bit */
  for (i = 1; i <= len; i++)
    {
      txbuf[i] = 0xFF;
    }

  ret = spi_manager_transfer(devid, txbuf, rxbuf, len + 1);
  if (ret == OK)
    {
      memcpy(buffer, &rxbuf[1], len);
    }

  free(txbuf);
  free(rxbuf);

  return ret;
}

/****************************************************************************
 * Name: spi_manager_write_reg
 ****************************************************************************/

int spi_manager_write_reg(int devid, uint8_t reg, uint8_t value)
{
  uint8_t txbuf[2];

  txbuf[0] = reg & 0x7F;  /* Clear read bit */
  txbuf[1] = value;

  return spi_manager_transfer(devid, txbuf, NULL, 2);
}

/****************************************************************************
 * Name: spi_manager_write_regs
 ****************************************************************************/

int spi_manager_write_regs(int devid, uint8_t reg,
                           const uint8_t *buffer, size_t len)
{
  uint8_t *txbuf;
  int ret;

  if (!buffer || len == 0)
    {
      return -EINVAL;
    }

  txbuf = malloc(len + 1);
  if (!txbuf)
    {
      return -ENOMEM;
    }

  txbuf[0] = reg & 0x7F;  /* Clear read bit */
  memcpy(&txbuf[1], buffer, len);

  ret = spi_manager_transfer(devid, txbuf, NULL, len + 1);

  free(txbuf);

  return ret;
}