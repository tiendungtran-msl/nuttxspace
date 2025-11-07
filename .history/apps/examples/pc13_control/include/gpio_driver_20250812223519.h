/****************************************************************************
 * include/gpio_driver.h
 ****************************************************************************/

#ifndef __GPIO_DRIVER_H
#define __GPIO_DRIVER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* GPIO initialization and control */
int gpio_pc13_init(void);
int gpio_pc13_deinit(void);
int gpio_pc13_set(bool state);
bool gpio_pc13_get(void);
int gpio_pc13_toggle(void);

#endif /* __GPIO_DRIVER_H */