/****************************************************************************
 * apps/examples/icm42688p_all/icm42688p_spi.c
 *
 * Minimal SPI user-space helper for ICM42688P WHO_AM_I test
 *
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/ioctl.h>

#include <nuttx/spi/spi.h>
#include <nuttx/spi/spi_transfer.h>

/* ICM42688P registers */
#define ICM42688P_WHO_AM_I       0x75
#define ICM42688P_SPI_READ       0x80
#define ICM42688P_DEVICE_ID      0x47

/* SPI settings for ICM42688P */
#define ICM42688P_SPI_MODE       SPIDEV_MODE3
#define ICM42688P_SPI_FREQUENCY  10000000  /* 10 MHz */
#define ICM42688P_SPI_BITS       8

/* Simple helper: open /dev/spiX and configure */
static int icm42688p_spi_init(const char *devpath, int *outfd)
{
  int fd;
  unsigned long arg;
  int ret;

  fd = open(devpath, O_RDWR);
  if (fd < 0)
    {
      fprintf(stderr, "open(%s) failed: %s\n", devpath, strerror(errno));
      return -errno;
    }

  /* Set mode */
  arg = (unsigned long)ICM42688P_SPI_MODE;
  ret = ioctl(fd, SPI_SETMODE, arg);
  if (ret < 0)
    {
      fprintf(stderr, "SPIIOC_SETMODE failed: %s\n", strerror(errno));
      close(fd);
      return -errno;
    }

  /* Set frequency */
  arg = (unsigned long)ICM42688P_SPI_FREQUENCY;
  ret = ioctl(fd, SPIIOC_SETFREQUENCY, arg);
  if (ret < 0)
    {
      fprintf(stderr, "SPIIOC_SETFREQUENCY failed: %s\n", strerror(errno));
      close(fd);
      return -errno;
    }

  /* Set bits */
  arg = (unsigned long)ICM42688P_SPI_BITS;
  ret = ioctl(fd, SPIIOC_SETBITS, arg);
  if (ret < 0)
    {
      fprintf(stderr, "SPIIOC_SETBITS failed: %s\n", strerror(errno));
      close(fd);
      return -errno;
    }

  *outfd = fd;
  return OK;
}

/* Read single register using SPIIOC_TRANSFER */
static int icm42688p_read_register(int fd, uint8_t reg, uint8_t *val)
{
  int ret;
  uint8_t tx[2];
  uint8_t rx[2];

  memset(tx, 0xff, sizeof(tx));
  tx[0] = reg | ICM42688P_SPI_READ;

  /* Prepare one transfer */
  struct spi_trans_s trans =
  {
    .deselect = true,
    .delay    = 0,
    .nwords   = 2,
    .txbuffer = tx,
    .rxbuffer = rx
  };

  struct spi_sequence_s seq =
  {
    .ntrans = 1,
    .trans  = &trans
  };

  ret = ioctl(fd, SPIIOC_TRANSFER, (unsigned long)&seq);
  if (ret < 0)
    {
      fprintf(stderr, "SPIIOC_TRANSFER failed: %s\n", strerror(errno));
      return -errno;
    }

  /* rx[0] is dummy/read status, rx[1] contains the register value */
  *val = rx[1];
  return OK;
}

/* Read multiple bytes starting at reg using SPIIOC_TRANSFER */
static int icm42688p_read_registers(int fd, uint8_t reg, uint8_t *buf, size_t len)
{
  int ret;
  size_t i;

  /* Build tx buffer: first byte is reg|read, then len dummy bytes */
  uint8_t *tx = malloc(len + 1);
  uint8_t *rx = malloc(len + 1);
  if (!tx || !rx)
    {
      free(tx);
      free(rx);
      return -ENOMEM;
    }
  tx[0] = reg | ICM42688P_SPI_READ;
  for (i = 1; i <= len; i++)
    {
      tx[i] = 0xFF;
    }

  struct spi_trans_s trans =
  {
    .deselect = true,
    .delay    = 0,
    .nwords   = (unsigned int)(len + 1),
    .txbuffer = tx,
    .rxbuffer = rx
  };

  struct spi_sequence_s seq =
  {
    .ntrans = 1,
    .trans  = &trans
  };

  ret = ioctl(fd, SPIIOC_TRANSFER, (unsigned long)&seq);
  if (ret < 0)
    {
      fprintf(stderr, "SPIIOC_TRANSFER failed: %s\n", strerror(errno));
      free(tx);
      free(rx);
      return -errno;
    }

  /* Copy output bytes (skip rx[0] which corresponds to the COMMAND) */
  for (i = 0; i < len; i++)
    {
      buf[i] = rx[i + 1];
    }

  free(tx);
  free(rx);
  return OK;
}

/* Minimal test program */
int main(int argc, FAR char *argv[])
{
  int fd;
  char devpath[32];
  uint8_t who = 0;
  int ret;
  int sensor_id = 0;

  if (argc >= 2)
    {
      sensor_id = atoi(argv[1]);
    }

  /* Map sensor_id to device path.
   * Default mapping used earlier: /dev/spi0..spi3
   */
  snprintf(devpath, sizeof(devpath), "/dev/spi%d", sensor_id);

  ret = icm42688p_spi_init(devpath, &fd);
  if (ret < 0)
    {
      fprintf(stderr, "Failed to init SPI device %s: %d\n", devpath, ret);
      return ret;
    }

  printf("SPI device %s opened and configured\n", devpath);

  /* Read WHO_AM_I */
  ret = icm42688p_read_register(fd, ICM42688P_WHO_AM_I, &who);
  if (ret < 0)
    {
      fprintf(stderr, "Failed to read WHO_AM_I: %d\n", ret);
      close(fd);
      return ret;
    }

  printf("WHO_AM_I = 0x%02X\n", who);
  if (who == ICM42688P_DEVICE_ID)
    {
      printf("ICM42688P detected OK\n");
    }
  else
    {
      printf("Unexpected WHO_AM_I (expected 0x%02X)\n", ICM42688P_DEVICE_ID);
    }

  close(fd);
  return 0;
}