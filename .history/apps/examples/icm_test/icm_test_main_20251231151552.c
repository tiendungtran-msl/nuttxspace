/****************************************************************************
 * apps/examples/icm_test/icm_test_main.c
 *
 * Test application for 4x ICM42688P IMUs on SPI1
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

#include <nuttx/spi/spi.h>
#include <arch/board/board.h>

#include "stm32_spi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ICM42688P Register Map (Bank 0) */
#define ICM42688_REG_DEVICE_CONFIG      0x11
#define ICM42688_REG_DRIVE_CONFIG       0x13
#define ICM42688_REG_INT_CONFIG         0x14
#define ICM42688_REG_FIFO_CONFIG        0x16
#define ICM42688_REG_TEMP_DATA1         0x1D
#define ICM42688_REG_TEMP_DATA0         0x1E
#define ICM42688_REG_ACCEL_DATA_X1      0x1F
#define ICM42688_REG_ACCEL_DATA_X0      0x20
#define ICM42688_REG_GYRO_DATA_X1       0x25
#define ICM42688_REG_PWR_MGMT0          0x4E
#define ICM42688_REG_GYRO_CONFIG0       0x4F
#define ICM42688_REG_ACCEL_CONFIG0      0x50
#define ICM42688_REG_WHO_AM_I           0x75
#define ICM42688_REG_BANK_SEL           0x76

/* ICM42688P Constants */
#define ICM42688_WHOAMI_VALUE           0x47
#define ICM42688_SPI_READ               0x80

/* SPI Configuration */
#define ICM42688_SPI_BUS                1
#define ICM42688_SPI_FREQUENCY          1000000  /* 1 MHz for initial access */
#define ICM42688_SPI_MODE               SPIDEV_MODE3

/* Number of IMUs */
#define NUM_IMUS                        4

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct imu_data_s
{
  int16_t accel_x;
  int16_t accel_y;
  int16_t accel_z;
  int16_t gyro_x;
  int16_t gyro_y;
  int16_t gyro_z;
  int16_t temp_raw;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static FAR struct spi_dev_s *g_spi1 = NULL;

static const uint32_t g_imu_devids[NUM_IMUS] =
{
    SPIDEV_ICM0,
    SPIDEV_ICM1,
    SPIDEV_ICM2,
    SPIDEV_ICM3
};

static const char *g_imu_names[NUM_IMUS] =
{
  "IMU1",
  "IMU2",
  "IMU3",
  "IMU4"
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: icm42688_read_reg
 ****************************************************************************/

static uint8_t icm42688_read_reg(FAR struct spi_dev_s *spi, 
                                  uint32_t devid, 
                                  uint8_t reg)
{
  uint8_t tx[2];
  uint8_t rx[2];

  tx[0] = reg | ICM42688_SPI_READ;
  tx[1] = 0x00;

  SPI_LOCK(spi, true);
  SPI_SETMODE(spi, ICM42688_SPI_MODE);
  SPI_SETBITS(spi, 8);
  SPI_SETFREQUENCY(spi, ICM42688_SPI_FREQUENCY);

  SPI_SELECT(spi, devid, true);
  SPI_EXCHANGE(spi, tx, rx, 2);
  SPI_SELECT(spi, devid, false);

  SPI_LOCK(spi, false);

  return rx[1];
}

/****************************************************************************
 * Name: icm42688_write_reg
 ****************************************************************************/

static void icm42688_write_reg(FAR struct spi_dev_s *spi,
                                uint32_t devid,
                                uint8_t reg,
                                uint8_t value)
{
  uint8_t tx[2];

  tx[0] = reg & 0x7F;  /* Write bit = 0 */
  tx[1] = value;

  SPI_LOCK(spi, true);
  SPI_SETMODE(spi, ICM42688_SPI_MODE);
  SPI_SETBITS(spi, 8);
  SPI_SETFREQUENCY(spi, ICM42688_SPI_FREQUENCY);

  SPI_SELECT(spi, devid, true);
  SPI_SEND(spi, tx[0]);
  SPI_SEND(spi, tx[1]);
  SPI_SELECT(spi, devid, false);

  SPI_LOCK(spi, false);
}

/****************************************************************************
 * Name:  icm42688_read_burst
 ****************************************************************************/

static void icm42688_read_burst(FAR struct spi_dev_s *spi,
                                 uint32_t devid,
                                 uint8_t start_reg,
                                 FAR uint8_t *buffer,
                                 size_t len)
{
  uint8_t tx_cmd = start_reg | ICM42688_SPI_READ;

  SPI_LOCK(spi, true);
  SPI_SETMODE(spi, ICM42688_SPI_MODE);
  SPI_SETBITS(spi, 8);
  SPI_SETFREQUENCY(spi, ICM42688_SPI_FREQUENCY);

  SPI_SELECT(spi, devid, true);
  
  /* Send read command */
  SPI_SEND(spi, tx_cmd);
  
  /* Read data */
  SPI_RECVBLOCK(spi, buffer, len);
  
  SPI_SELECT(spi, devid, false);

  SPI_LOCK(spi, false);
}

/****************************************************************************
 * Name: icm42688_check_whoami
 ****************************************************************************/

static int icm42688_check_whoami(FAR struct spi_dev_s *spi,
                                  uint32_t devid,
                                  FAR const char *name)
{
  uint8_t whoami;

  whoami = icm42688_read_reg(spi, devid, ICM42688_REG_WHO_AM_I);

  printf("  %s: WHO_AM_I = 0x%02X ", name, whoami);

  if (whoami == ICM42688_WHOAMI_VALUE)
    {
      printf("[OK]\n");
      return OK;
    }
  else
    {
      printf("[FAIL - expected 0x47]\n");
      return -ENODEV;
    }
}

/****************************************************************************
 * Name: icm42688_init
 ****************************************************************************/

static int icm42688_init(FAR struct spi_dev_s *spi,
                         uint32_t devid,
                         FAR const char *name)
{
  printf("  %s: Initializing.. .\n", name);

  /* Soft reset */
  icm42688_write_reg(spi, devid, ICM42688_REG_DEVICE_CONFIG, 0x01);
  usleep(1000);  /* Wait 1ms for reset */

  /* Wait for reset to complete */
  int retry = 10;
  while (retry-- > 0)
    {
      uint8_t reg = icm42688_read_reg(spi, devid, ICM42688_REG_WHO_AM_I);
      if (reg == ICM42688_WHOAMI_VALUE)
        {
          break;
        }
      usleep(1000);
    }

  if (retry <= 0)
    {
      printf("  %s: Reset timeout\n", name);
      return -ETIMEDOUT;
    }

  /* Power management:  turn on accel and gyro */
  icm42688_write_reg(spi, devid, ICM42688_REG_PWR_MGMT0, 0x0F);
  usleep(1000);

  /* Configure gyro:  ±2000 dps, ODR=1kHz */
  icm42688_write_reg(spi, devid, ICM42688_REG_GYRO_CONFIG0, 0x06);

  /* Configure accel: ±16g, ODR=1kHz */
  icm42688_write_reg(spi, devid, ICM42688_REG_ACCEL_CONFIG0, 0x06);

  usleep(50000);  /* Wait 50ms for sensors to stabilize */

  printf("  %s: Initialization complete\n", name);
  return OK;
}

/****************************************************************************
 * Name: icm42688_read_data
 ****************************************************************************/

static int icm42688_read_data(FAR struct spi_dev_s *spi,
                              uint32_t devid,
                              FAR struct imu_data_s *data)
{
  uint8_t buffer[14];

  /* Read temperature + accel + gyro in one burst */
  icm42688_read_burst(spi, devid, ICM42688_REG_TEMP_DATA1, buffer, 14);

  /* Parse data (big endian) */
  data->temp_raw  = (int16_t)((buffer[0] << 8) | buffer[1]);
  data->accel_x   = (int16_t)((buffer[2] << 8) | buffer[3]);
  data->accel_y   = (int16_t)((buffer[4] << 8) | buffer[5]);
  data->accel_z   = (int16_t)((buffer[6] << 8) | buffer[7]);
  data->gyro_x    = (int16_t)((buffer[8] << 8) | buffer[9]);
  data->gyro_y    = (int16_t)((buffer[10] << 8) | buffer[11]);
  data->gyro_z    = (int16_t)((buffer[12] << 8) | buffer[13]);

  return OK;
}

/****************************************************************************
 * Name: print_usage
 ****************************************************************************/

static void print_usage(FAR const char *progname)
{
  printf("\nUsage: %s <test_mode>\n", progname);
  printf("\nTest modes:\n");
  printf("  0 - Scan all 4 IMUs (WHO_AM_I check)\n");
  printf("  1 - Initialize all 4 IMUs\n");
  printf("  2 - Read data from all 4 IMUs (once)\n");
  printf("  3 - Continuous read (10Hz, press Ctrl+C to stop)\n");
  printf("  4 - Read data from IMU1 only\n");
  printf("  5 - Read data from IMU2 only\n");
  printf("  6 - Read data from IMU3 only\n");
  printf("  7 - Read data from IMU4 only\n");
  printf("  8 - Performance test (1000 reads)\n");
  printf("\n");
}

/****************************************************************************
 * Name: test_scan_all
 ****************************************************************************/

static void test_scan_all(void)
{
  int i;

  printf("\n=== Scanning 4 IMUs on SPI1 ===\n");

  for (i = 0; i < NUM_IMUS; i++)
    {
      icm42688_check_whoami(g_spi1, g_imu_devids[i], g_imu_names[i]);
    }

  printf("\n");
}

/****************************************************************************
 * Name: test_init_all
 ****************************************************************************/

static void test_init_all(void)
{
  int i;

  printf("\n=== Initializing 4 IMUs ===\n");

  for (i = 0; i < NUM_IMUS; i++)
    {
      if (icm42688_init(g_spi1, g_imu_devids[i], g_imu_names[i]) < 0)
        {
          printf("  %s:  Initialization failed\n", g_imu_names[i]);
        }
    }

  printf("\n");
}

/****************************************************************************
 * Name: test_read_all_once
 ****************************************************************************/

static void test_read_all_once(void)
{
  int i;
  struct imu_data_s data;

  printf("\n=== Reading all 4 IMUs (once) ===\n");

  for (i = 0; i < NUM_IMUS; i++)
    {
      if (icm42688_read_data(g_spi1, g_imu_devids[i], &data) == OK)
        {
          printf("%s: ", g_imu_names[i]);
          printf("A[%6d %6d %6d] ", data.accel_x, data.accel_y, data. accel_z);
          printf("G[%6d %6d %6d] ", data. gyro_x, data.gyro_y, data.gyro_z);
          printf("T[%6d]\n", data.temp_raw);
        }
      else
        {
          printf("%s: Read failed\n", g_imu_names[i]);
        }
    }

  printf("\n");
}

/****************************************************************************
 * Name: test_continuous_read
 ****************************************************************************/

static void test_continuous_read(void)
{
  struct imu_data_s data[NUM_IMUS];
  int count = 0;

  printf("\n=== Continuous read (10Hz, Ctrl+C to stop) ===\n");
  printf("Format:  Accel[X Y Z] Gyro[X Y Z] Temp[raw]\n\n");

  while (true)
    {
      printf("\n--- Sample #%d ---\n", ++count);

      for (int i = 0; i < NUM_IMUS; i++)
        {
          if (icm42688_read_data(g_spi1, g_imu_devids[i], &data[i]) == OK)
            {
              printf("%s: ", g_imu_names[i]);
              printf("A[%6d %6d %6d] ", 
                     data[i].accel_x, data[i]. accel_y, data[i].accel_z);
              printf("G[%6d %6d %6d] ",
                     data[i].gyro_x, data[i]. gyro_y, data[i].gyro_z);
              printf("T[%6d]\n", data[i].temp_raw);
            }
        }

      usleep(100000);  /* 100ms = 10Hz */
    }
}

/****************************************************************************
 * Name: test_single_imu
 ****************************************************************************/

static void test_single_imu(int imu_index)
{
  struct imu_data_s data;

  if (imu_index < 0 || imu_index >= NUM_IMUS)
    {
      printf("ERROR: Invalid IMU index %d\n", imu_index);
      return;
    }

  printf("\n=== Reading %s only ===\n", g_imu_names[imu_index]);

  if (icm42688_read_data(g_spi1, g_imu_devids[imu_index], &data) == OK)
    {
      printf("Accel:  X=%6d Y=%6d Z=%6d\n",
             data.accel_x, data. accel_y, data.accel_z);
      printf("Gyro:   X=%6d Y=%6d Z=%6d\n",
             data.gyro_x, data.gyro_y, data.gyro_z);
      printf("Temp:  %6d (raw)\n", data.temp_raw);
    }
  else
    {
      printf("Read failed\n");
    }

  printf("\n");
}

/****************************************************************************
 * Name:  test_performance
 ****************************************************************************/

static void test_performance(void)
{
  struct imu_data_s data;
  struct timespec start, end;
  uint32_t elapsed_ms;
  int i, j;
  int iterations = 1000;

  printf("\n=== Performance test (%d reads per IMU) ===\n", iterations);

  for (i = 0; i < NUM_IMUS; i++)
    {
      clock_gettime(CLOCK_MONOTONIC, &start);

      for (j = 0; j < iterations; j++)
        {
          icm42688_read_data(g_spi1, g_imu_devids[i], &data);
        }

      clock_gettime(CLOCK_MONOTONIC, &end);

      elapsed_ms = (end. tv_sec - start.tv_sec) * 1000 +
                   (end.tv_nsec - start.tv_nsec) / 1000000;

      printf("%s: %d reads in %u ms (%.1f Hz)\n",
             g_imu_names[i],
             iterations,
             elapsed_ms,
             (float)iterations * 1000.0f / elapsed_ms);
    }

  printf("\n");
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int test_mode;

  printf("\n========================================\n");
  printf("ICM42688P Test (4 IMUs on SPI1)\n");
  printf("========================================\n");

  /* Initialize SPI bus */
  g_spi1 = stm32_spibus_initialize(ICM42688_SPI_BUS);
  if (! g_spi1)
    {
      printf("ERROR: Failed to initialize SPI1\n");
      return EXIT_FAILURE;
    }

  printf("SPI1 bus initialized\n");

  /* Parse command line */
  if (argc < 2)
    {
      print_usage(argv[0]);
      return EXIT_FAILURE;
    }

  test_mode = atoi(argv[1]);

  /* Execute test */
  switch (test_mode)
    {
      case 0:
        test_scan_all();
        break;

      case 1:
        test_init_all();
        break;

      case 2:
        test_read_all_once();
        break;

      case 3:
        test_continuous_read();
        break;

      case 4:
        test_single_imu(0);
        break;

      case 5:
        test_single_imu(1);
        break;

      case 6:
        test_single_imu(2);
        break;

      case 7:
        test_single_imu(3);
        break;

      case 8:
        test_performance();
        break;

      default: 
        printf("ERROR: Invalid test mode %d\n", test_mode);
        print_usage(argv[0]);
        return EXIT_FAILURE;
    }

  return EXIT_SUCCESS;
}