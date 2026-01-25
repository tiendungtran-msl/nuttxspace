#include <stdio.h>
#include <unistd.h>
#include <nuttx/i2c/i2c_master.h>
#include "../drivers/sensors/bmm150/bmm150_driver.h"
#include "../drivers/sensors/bmm150/bmm150_regs.h"
#include "bmm150_test.h"

#define BMM150_TEST_I2C_ADDR      BMM150_I2C_ADDRESS_CSB_HIGH_SDO_HIGH
#define BMM150_TEST_NUM_SAMPLES   10

static void bmm150_test_print_direction(float yaw)
{
  const char *dir = "Unknown";
  if (yaw >= 337.5f || yaw < 22.5f)
    dir = "North";
  else if (yaw < 67.5f)  dir = "North-East";
  else if (yaw < 112.5f) dir = "East";
  else if (yaw < 157.5f) dir = "South-East";
  else if (yaw < 202.5f) dir = "South";
  else if (yaw < 247.5f) dir = "South-West";
  else if (yaw < 292.5f) dir = "West";
  else if (yaw < 337.5f)dir = "North-West";
  printf("  -> Direction: %s\n", dir);
}

int bmm150_yaw_test(void)
{
  bmm150_dev_t dev;
  int ret;

  ret = bmm150_init(&dev, BMM150_TEST_I2C_ADDR);
  if (ret != BMM150_OK)
    {
      printf("BMM150 init error: %d\n", ret);
      return ret;
    }
  printf("BMM150 init: OK\n");

  for (int i = 0; i < BMM150_TEST_NUM_SAMPLES; i++)
    {
      float yaw;
      ret = bmm150_get_yaw(&dev, &yaw);
      if (ret == BMM150_OK)
        {
          printf("Sample %02d: Yaw = %7.2f deg\n", i+1, yaw);
          bmm150_test_print_direction(yaw);
        }
      else
        {
          printf("Read Yaw error: %d\n", ret);
        }
      usleep(200000); // 200 ms
    }
  bmm150_deinit(&dev);
  printf("[BMM150 Test] Done!\n");

  return 0;
}