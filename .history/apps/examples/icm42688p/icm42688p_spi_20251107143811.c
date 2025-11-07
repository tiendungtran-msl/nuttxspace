/****************************************************************************
 * apps/examples/icm42688p_all/icm42688p_spi.c
 *
 * Minimal SPI user-space helper for ICM42688P WHO_AM_I test
 *
 * Note: SPI configuration (mode, frequency, bits) must be set in the
 * driver or board configuration, not via user-space ioctl.
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

/* Simple helper: open /dev/spiX */
static int icm42688p_spi_init(const char *devpath, int *outfd)
{
  int fd;

  fd = open(devpath, O_RDWR);
  if (fd < 0)
    {
      fprintf(stderr, "open(%s) failed: %s\n", devpath, strerror(errno));
      return -errno;
    }

  printf("SPI device %s opened successfully\n", devpath);
  printf("Note: SPI mode/frequency/bits must be configured in driver\n");

  *outfd = fd;
  return OK;
}

/* Read single register using SPIIOC_TRANSFER */
static int icm42688p_read_register(int fd, uint8_t reg, uint8_t *val)
{
  int ret;
  uint8_t tx[2];
  uint8_t rx[2];

  memset(rx, 0, sizeof(rx));
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

  memset(rx, 0, len + 1);
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
   * Default mapping: /dev/spi0..spi3
   */
  snprintf(devpath, sizeof(devpath), "/dev/spi%d", sensor_id);

  printf("ICM42688P SPI Test\n");
  printf("==================\n");
  printf("Device: %s\n", devpath);
  printf("Expected Device ID: 0x%02X\n\n", ICM42688P_DEVICE_ID);

  ret = icm42688p_spi_init(devpath, &fd);
  if (ret < 0)
    {
      fprintf(stderr, "Failed to init SPI device %s: %d\n", devpath, ret);
      return ret;
    }

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
      printf("✓ ICM42688P detected successfully!\n");
    }
  else
    {
      printf("✗ Unexpected WHO_AM_I (expected 0x%02X, got 0x%02X)\n", 
             ICM42688P_DEVICE_ID, who);
    }

  close(fd);
  return (who == ICM42688P_DEVICE_ID) ? 0 : 1;
}