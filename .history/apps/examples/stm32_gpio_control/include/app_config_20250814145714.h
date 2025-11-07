/****************************************************************************
 * examples/stm32_gpio_control/include/app_config.h
 *
 * Application Configuration Header
 * Định nghĩa các cấu hình chung cho ứng dụng
 ****************************************************************************/

#ifndef __EXAMPLES_STM32_GPIO_CONTROL_APP_CONFIG_H
#define __EXAMPLES_STM32_GPIO_CONTROL_APP_CONFIG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* GPIO Configuration - STM32F411 BlackPill PC13 (Built-in LED) */
#define APP_GPIO_PORT           2    /* Port C = 2 (A=0, B=1, C=2, ...) */
#define APP_GPIO_PIN            13   /* Pin 13 */
#define APP_GPIO_DEVICE_PATH    "/dev/gpio13"

/* Application Timing */
#define APP_BLINK_DELAY_MS      1000 /* 1 second blink interval */
#define APP_LOOP_COUNT          10   /* Number of blinks */

/* Debug Configuration */
#ifdef CONFIG_DEBUG_GPIO
#  define gpio_info(format, ...)  printf("GPIO: " format, ##__VA_ARGS__)
#else
#  define gpio_info(format, ...)
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Application State */
typedef enum
{
  APP_STATE_INIT = 0,
  APP_STATE_RUNNING,
  APP_STATE_STOPPED,
  APP_STATE_ERROR
} app_state_t;

/* GPIO Control Commands */
typedef enum
{
  GPIO_CMD_INIT = 0,
  GPIO_CMD_SET_HIGH,
  GPIO_CMD_SET_LOW,
  GPIO_CMD_TOGGLE,
  GPIO_CMD_READ,
  GPIO_CMD_CLEANUP
} gpio_cmd_t;

#endif /* __EXAMPLES_STM32_GPIO_CONTROL_APP_CONFIG_H */