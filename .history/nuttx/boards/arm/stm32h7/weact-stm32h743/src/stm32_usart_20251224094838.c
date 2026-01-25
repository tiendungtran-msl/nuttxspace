/****************************************************************************
 * boards/arm/stm32h7/weact-stm32h743/src/weact_board_serial.c
 *
 * UART8 console: PE0(RX), PE1(TX)
 * UART4: PA0(TX), PA1(RX)
 *
 * In mainline NuttX STM32H7, UART GPIO alternate function configuration is
 * typically handled by the STM32 serial driver when the corresponding
 * CONFIG_USARTx_* options are enabled.
 *
 * Keep this file as a placeholder to centralize any future board-specific
 * serial pin init or RS-485/GPS power enable handling.
 ****************************************************************************/

#include <nuttx/config.h>

int weact_board_serial_init(void)
{
  /* Nothing required here for typical mainline STM32H7 serial.
   * If your tree requires explicit GPIO setup, add it here.
   */
  return 0;
}