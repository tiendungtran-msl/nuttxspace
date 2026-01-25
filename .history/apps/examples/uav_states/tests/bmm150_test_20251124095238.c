/****************************************************************************
 * apps/examples/uav_states/drivers/sensors/bmm150/bmm150_test.c
 *
 * BMM150 Magnetometer Simple Test Program (Yaw only)
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <nuttx/i2c/i2c_master.h>

#include "bmm150_driver.h"
#include "bmm150_regs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BMM150_I2C_BUS          1
#define BMM150_I2C_ADDR         BMM150_DEFAULT_I2C_ADDRESS

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: print_compass_direction
 *
 * Description:
 *   Convert yaw angle to compass direction
 *
 ****************************************************************************/

static void print_compass_direction(float yaw)
{
  const char *direction;

  if (yaw >= 337.5f || yaw < 22.5f)
    direction = "North (N)";
  else if (yaw >= 22.5f && yaw < 67.5f)
    direction = "North-East (NE)";
  else if (yaw >= 67.5f && yaw < 112.5f)
    direction = "East (E)";
  else if (yaw >= 112.5f && yaw < 157.5f)
    direction = "South-East (SE)";
  else if (yaw >= 157.5f && yaw < 202.5f)
    direction = "South (S)";
  else if (yaw >= 202.5f && yaw < 247.5f)
    direction = "South-West (SW)";
  else if (yaw >= 247.5f && yaw < 292.5f)
    direction = "West (W)";
  else
    direction = "North-West (NW)";

  printf("  Direction: %s\n", direction);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: main
 *
 * Description:
 *   BMM150 simple test program - Yaw angle only
 *
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  bmm150_dev_t dev;
  bmm150_mag_data_t mag_data;
  float yaw;
  int ret;
  int i;
  int num_samples = 20;  /* Default 20 samples */

  printf("\n");
  printf("========================================\n");
  printf("  BMM150 Yaw Angle Test\n");
  printf("========================================\n");

  /* Get number of samples from command line */
  if (argc > 1)
    {
      num_samples = atoi(argv[1]);
      if (num_samples <= 0)
        {
          num_samples = 20;
        }
    }

  /* Initialize I2C */
  printf("\nInitializing I2C bus %d...\n", BMM150_I2C_BUS);
  dev.i2c = i2c_init(BMM150_I2C_BUS);
  if (!dev.i2c)
    {
      printf("ERROR: Failed to initialize I2C bus\n");
      return -1;
    }

  /* Initialize BMM150 */
  printf("Initializing BMM150 at address 0x%02X...\n", BMM150_I2C_ADDR);
  ret = bmm150_init(&dev, BMM150_I2C_ADDR);
  if (ret != BMM150_OK)
    {
      printf("ERROR: BMM150 initialization failed (error: %d)\n", ret);
      return -1;
    }

  printf("BMM150 initialized successfully!\n");
  bmm150_print_status(&dev);

  /* Read and display yaw angles */
  printf("\n========================================\n");
  printf("Reading %d samples...\n", num_samples);
  printf("========================================\n\n");

  for (i = 0; i < num_samples; i++)
    {
      /* Read magnetometer data */
      ret = bmm150_read_mag(&dev, &mag_data);
      if (ret != BMM150_OK)
        {
          printf("Sample %d: Read error (%d)\n", i + 1, ret);
          continue;
        }

      /* Get yaw angle */
      ret = bmm150_get_yaw(&dev, &yaw);
      if (ret != BMM150_OK)
        {
          printf("Sample %d: Yaw calculation error (%d)\n", i + 1, ret);
          continue;
        }

      /* Display results */
      printf("Sample %3d:\n", i + 1);
      printf("  Mag X: %8.2f µT\n", mag_data.x);
      printf("  Mag Y: %8.2f µT\n", mag_data.y);
      printf("  Mag Z: %8.2f µT\n", mag_data.z);
      printf("  Yaw:   %6.2f° (0-360°)\n", yaw);
      print_compass_direction(yaw);
      printf("\n");

      /* Wait 200ms between samples */
      usleep(200000);
    }

  /* Final statistics */
  printf("========================================\n");
  printf("Test Complete\n");
  printf("========================================\n");
  bmm150_print_status(&dev);

  /* Cleanup */
  printf("\nCleaning up...\n");
  bmm150_deinit(&dev);

  printf("Done!\n\n");

  return 0;
}