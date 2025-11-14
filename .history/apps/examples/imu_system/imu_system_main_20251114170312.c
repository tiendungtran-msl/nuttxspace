/****************************************************************************
 * apps/examples/imu_system/imu_system_main.c
 * 
 * Simple I2C BMM150 Test
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "drivers/i2c/i2c_manager.h"
#include "drivers/bmm150/bmm150_driver.h"

int main(int argc, FAR char *argv[])
{
  int ret;
  
  printf("IMU System - I2C BMM150 Test\n");
  
  /* Initialize I2C */
  ret = i2c_manager_init(1);
  if (ret < 0)
    {
      printf("ERROR: I2C init failed: %d\n", ret);
      return EXIT_FAILURE;
    }
  
  printf("I2C initialized\n");
  
  /* Initialize BMM150 */
  ret = bmm150_init();
  if (ret < 0)
    {
      printf("ERROR: BMM150 init failed: %d\n", ret);
      i2c_manager_deinit();
      return EXIT_FAILURE;
    }
  
  printf("BMM150 initialized\n");
  printf("Success!\n");
  
  /* Cleanup */
  bmm150_deinit();
  i2c_manager_deinit();
  
  return EXIT_SUCCESS;
}