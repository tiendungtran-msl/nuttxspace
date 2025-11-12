/****************************************************************************
 * apps/examples/imu_system/drivers/led/led_driver.h
 *
 * LED Driver for Status Indication
 *
 ****************************************************************************/

#ifndef __APPS_EXAMPLES_IMU_SYSTEM_DRIVERS_LED_DRIVER_H
#define __APPS_EXAMPLES_IMU_SYSTEM_DRIVERS_LED_DRIVER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: led_init
 *
 * Description:
 *   Initialize LED driver (GPIO PC0)
 *
 * Returned Value:
 *   0 on success, negative errno on failure
 *
 ****************************************************************************/

int led_init(void);

/****************************************************************************
 * Name: led_deinit
 *
 * Description:
 *   Deinitialize LED driver
 *
 ****************************************************************************/

void led_deinit(void);

/****************************************************************************
 * Name: led_set
 *
 * Description:
 *   Set LED state
 *
 * Input Parameters:
 *   on - true to turn LED on, false to turn off
 *
 ****************************************************************************/

void led_set(bool on);

/****************************************************************************
 * Name: led_blink
 *
 * Description:
 *   Set LED to blink at specified period
 *
 * Input Parameters:
 *   period_ms - Blink period in milliseconds (0 to stop blinking)
 *
 ****************************************************************************/

void led_blink(uint32_t period_ms);

/****************************************************************************
 * Name: led_update
 *
 * Description:
 *   Update LED state (call periodically from LED task)
 *
 ****************************************************************************/

void led_update(void);

#endif /* __APPS_EXAMPLES_IMU_SYSTEM_DRIVERS_LED_DRIVER_H */
