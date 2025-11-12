/****************************************************************************
 * apps/examples/imu_system/drivers/spi/spi_manager.c
 *
 * SPI Bus Manager implementation
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <debug.h>
#include <string.h>
#include <pthread.h>
#include <nuttx/spi/spi.h>
#include <nuttx/spi/spi_transfer.h>

#include "spi_manager.h"
#include "../../utils/config.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SPI_READ_BIT 0x80

/****************************************************************************
 * Private Data
 ****************************************************************************/

static int g_spi_fd = -1;
static pthread_mutex_t g_spi_mutex = PTHREAD_MUTEX_INITIALIZER;

/****************************************************************************
 * External Functions
 ****************************************************************************/

/* Board-specific function for chip select control */

extern void board_spi1_sensor_select(uint8_t devid, bool selected);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spi_manager_init
 ****************************************************************************/

int spi_manager_init(int bus)
{
  char devpath[32];
  int ret;

  snprintf(devpath, sizeof(devpath), "/dev/spi%d", bus);

  g_spi_fd = open(devpath, O_RDWR);
  if (g_spi_fd < 0)
    {
      ret = -errno;
      snerr("ERROR: Failed to open %s: %d\n", devpath, ret);
      return ret;
    }

  /* Configure SPI mode and frequency */

  ret = ioctl(g_spi_fd, SPIIOC_SETMODE, SPIDEV_MODE3);
  if (ret < 0)
    {
      snerr("ERROR: Failed to set SPI mode\n");
      goto errout;
    }

  ret = ioctl(g_spi_fd, SPIIOC_SETFREQUENCY, IMU_SPI_FREQUENCY);
  if (ret < 0)
    {
      snerr("ERROR: Failed to set SPI frequency\n");
      goto errout;
    }

  sninfo("SPI manager initialized on %s at %d Hz\n",
         devpath, IMU_SPI_FREQUENCY);

  return OK;

errout:
  close(g_spi_fd);
  g_spi_fd = -1;
  return ret;
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
 * Name: spi_manager_lock
 ****************************************************************************/

int spi_manager_lock(void)
{
  return pthread_mutex_lock(&g_spi_mutex);
}

/****************************************************************************
 * Name: spi_manager_unlock
 ****************************************************************************/

void spi_manager_unlock(void)
{
  pthread_mutex_unlock(&g_spi_mutex);
}

/****************************************************************************
 * Name: spi_manager_transfer
 ****************************************************************************/

int spi_manager_transfer(uint8_t devid, const uint8_t *txbuf,
                         uint8_t *rxbuf, size_t len)
{
  struct spi_trans_s trans;
  int ret;

  if (g_spi_fd < 0)
    {
      return -ENODEV;
    }

  if (devid >= SPI_MAX_DEVICES)
    {
      return -EINVAL;
    }

  /* Select device */

  board_spi1_sensor_select(devid, true);

  /* Setup transfer */

  memset(&trans, 0, sizeof(trans));
  trans.deselect = false;
  trans.txbuffer = (void *)txbuf;
  trans.rxbuffer = rxbuf;
  trans.nwords = len;

  /* Perform transfer */

  ret = ioctl(g_spi_fd, SPIIOC_TRANSFER, (unsigned long)&trans);

  /* Deselect device */

  board_spi1_sensor_select(devid, false);

  if (ret < 0)
    {
      ret = -errno;
      snerr("ERROR: SPI transfer failed: %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: spi_manager_write_reg
 ****************************************************************************/

int spi_manager_write_reg(uint8_t devid, uint8_t reg, uint8_t value)
{
  uint8_t txbuf[2];
  uint8_t rxbuf[2];

  txbuf[0] = reg & ~SPI_READ_BIT;
  txbuf[1] = value;

  return spi_manager_transfer(devid, txbuf, rxbuf, 2);
}

/****************************************************************************
 * Name: spi_manager_read_reg
 ****************************************************************************/

int spi_manager_read_reg(uint8_t devid, uint8_t reg, uint8_t *value)
{
  uint8_t txbuf[2];
  uint8_t rxbuf[2];
  int ret;

  txbuf[0] = reg | SPI_READ_BIT;
  txbuf[1] = 0;

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

int spi_manager_read_regs(uint8_t devid, uint8_t reg,
                          uint8_t *buffer, size_t len)
{
  uint8_t txbuf[64];
  uint8_t rxbuf[64];
  int ret;

  if (len > 63)
    {
      return -EINVAL;
    }

  /* First byte is register address with read bit */

  txbuf[0] = reg | SPI_READ_BIT;
  memset(&txbuf[1], 0, len);

  ret = spi_manager_transfer(devid, txbuf, rxbuf, len + 1);
  if (ret == OK)
    {
      memcpy(buffer, &rxbuf[1], len);
    }

  return ret;
}
