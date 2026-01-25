/****************************************************************************
 * boards/arm/stm32h7/weact-stm32h743/src/stm32_usart.c
 *
 * UART8 (console): PE0(RX), PE1(TX)
 * UART4: PA0(TX), PA1(RX)
 *
 * Typically, NuttX STM32H7 serial driver auto-configures pins based on
 * CONFIG_UARTx selections. This file is a placeholder for any board-specific
 * setup (RS-485 enable, power, etc.).
 ****************************************************************************/

#include <nuttx/config.h>

int weact_usart_initialize(void)
{
  /* Nothing needed for standard console/UART. 
   * If you need to enable GPS power rail or RS-485 direction pin, add here.
   */
  return 0;
}