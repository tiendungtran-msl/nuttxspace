/****************************************************************************
 * boards/arm/stm32h7/weact-stm32h743/src/stm32_i2c_bmm150.c
 *
 * BMM150 I2C Board Support
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/i2c/i2c_master.h>
#include "stm32_i2c.h"
#include "weact-stm32h743.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BMM150_I2C_BUS      1      /* I2C1 */
#define BMM150_I2C_ADDR     0x10   /* 7-bit address */
#define BMM150_I2C_FREQ     400000 /* 400 kHz */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_bmm150_i2c_initialize
 *
 * Description:
 *   Initialize I2C bus for BMM150 magnetometer
 ****************************************************************************/

struct i2c_master_s *board_bmm150_i2c_initialize(void)
{
  struct i2c_master_s *i2c;

  /* Get I2C1 instance */
  
  i2c = stm32_i2cbus_initialize(BMM150_I2C_BUS);
  if (i2c == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize I2C%d for BMM150\n",
             BMM150_I2C_BUS);
      return NULL;
    }

  syslog(LOG_INFO, "BMM150 I2C initialized on I2C%d at 0x%02x\n",
         BMM150_I2C_BUS, BMM150_I2C_ADDR);

  return i2c;
}