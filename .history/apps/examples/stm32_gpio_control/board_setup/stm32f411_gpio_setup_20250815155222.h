/****************************************************************************
 * examples/stm32_gpio_control/board_setup/stm32f411_gpio_setup.h
 *
 * Board-specific GPIO device setup for STM32F411-minimum
 * Tạo các device files /dev/gpio* cho board STM32F411 BlackPill
 ****************************************************************************/

#ifndef __EXAMPLES_STM32_GPIO_CONTROL_BOARD_SETUP_H
#define __EXAMPLES_STM32_GPIO_CONTROL_BOARD_SETUP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* GPIO Device Configuration for STM32F411 BlackPill */
#define BOARD_GPIO_PC13_DEVPATH    "/dev/gpio13"
#define BOARD_GPIO_PC13_GPOUT      "/dev/gpout13" 
#define BOARD_GPIO_PC13_GPIN       "/dev/gpin13"

/* GPIO Pin definitions for STM32F411 */
#define BOARD_GPIO_PC13_PIN        (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                                   GPIO_OUTPUT_CLEAR | GPIO_PORTC | GPIO_PIN13)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: board_gpio_setup
 *
 * Description:
 *   Setup board-specific GPIO devices
 *   - Register GPIO devices với NuttX device framework
 *   - Tạo character device files trong /dev/
 *   - Thường được gọi từ board initialization
 *
 * Returned Value:
 *   0 on success; a negated errno value on failure
 ****************************************************************************/

int board_gpio_setup(void);

/****************************************************************************
 * Name: board_gpio_cleanup  
 *
 * Description:
 *   Cleanup board GPIO devices
 *
 * Returned Value:
 *   0 on success; a negated errno value on failure
 ****************************************************************************/

int board_gpio_cleanup(void);

#config EXAMPLES_STM32_GPIO_CONTROL_BOARD_INIT
	bool "Initialize GPIO devices at board level"
	default y
	depends on EXAMPLES_STM32_GPIO_CONTROL
	---help---
		Setup GPIO devices during board initialization instead of application runtime.
		This creates /dev/gpio* devices early in boot process.

endif /* __EXAMPLES_STM32_GPIO_CONTROL_BOARD_SETUP_H */