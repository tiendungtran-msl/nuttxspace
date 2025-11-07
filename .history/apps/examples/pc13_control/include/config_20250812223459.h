/****************************************************************************
 * include/config.h
 ****************************************************************************/

#ifndef __PC13_CONTROL_CONFIG_H
#define __PC13_CONTROL_CONFIG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* GPIO Configuration */
#define PC13_PORT          GPIOC
#define PC13_PIN           GPIO_PIN13
#define PC13_GPIO_CONFIG   (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | GPIO_PORTC | GPIO_PIN13)

/* Application Configuration */
#ifndef CONFIG_PC13_CONTROL_BLINK_INTERVAL
#  define DEFAULT_BLINK_INTERVAL 1000  /* ms */
#else
#  define DEFAULT_BLINK_INTERVAL CONFIG_PC13_CONTROL_BLINK_INTERVAL
#endif

/* Command definitions */
#define CMD_BUFFER_SIZE    64
#define MAX_ARGS          8

/* Error codes */
#define PC13_OK           0
#define PC13_ERROR        -1
#define PC13_INVALID_ARG  -2

#endif /* __PC13_CONTROL_CONFIG_H */