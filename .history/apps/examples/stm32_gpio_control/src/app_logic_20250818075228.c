/****************************************************************************
 * examples/stm32_gpio_control/src/app_logic.c
 *
 * Application Logic Implementation
 * Triển khai logic ứng dụng - User layer
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <unistd.h>

#include "../include/gpio_driver.h"
#include "../include/app_config.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: app_logic_run_blink_test
 *
 * Description:
 *   Chạy test nhấp nhá LED trên PC13
 *   - Demonstration của layered architecture
 *   - User layer -> Driver layer -> HAL layer -> NuttX GPIO subsystem
 *
 * Input Parameters:
 *   ctx - Driver context
 *
 * Returned Value:
 *   0 on success, negative value on failure
 ****************************************************************************/

int app_logic_run_blink_test(gpio_driver_context_t *ctx)
{
  int ret;
  int i;
  app_state_t state;
  int error_count;
  uint32_t op_count;

  printf("\n=== STM32F411 BlackPill PC13 LED Blink Test ===\n");
  printf("Architecture: User -> Driver -> HAL -> NuttX GPIO\n");
  printf("Target: PC13 (Built-in LED)\n");
  printf("Blink count: %d, Interval: %d ms\n\n", 
         APP_LOOP_COUNT, APP_BLINK_DELAY_MS);

  /* Kiểm tra driver status */
  ret = gpio_driver_get_status(ctx, &state, &error_count, &op_count);
  if (ret != GPIO_DRIVER_OK || state != APP_STATE_RUNNING)
    {
      printf("ERROR: Driver not ready (state=%d, errors=%d)\n", 
             state, error_count);
      return -1;
    }

  /* Main blink loop */
  for (i = 0; i < APP_LOOP_COUNT; i++)
    {
      printf("[%d/%d] ", i + 1, APP_LOOP_COUNT);

      /* LED ON (PC13 is active LOW on BlackPill, so LOW = LED ON) */
      printf("LED ON  -> ");
      ret = gpio_driver_execute_command(ctx, GPIO_CMD_SET_LOW, false, NULL);
      if (ret != GPIO_DRIVER_OK)
        {
          printf("FAILED to turn LED ON\n");
          return -1;
        }
      printf("SUCCESS");

      /* Delay */
      usleep(APP_BLINK_DELAY_MS * 1000);

      /* LED OFF (PC13 HIGH = LED OFF) */
      printf(" | LED OFF -> ");
      ret = gpio_driver_execute_command(ctx, GPIO_CMD_SET_HIGH, true, NULL);
      if (ret != GPIO_DRIVER_OK)
        {
          printf("FAILED to turn LED OFF\n");
          return -1;
        }
      printf("SUCCESS\n");

      /* Delay */
      usleep(APP_BLINK_DELAY_MS * 1000);
    }

  /* Final status report */
  ret = gpio_driver_get_status(ctx, &state, &error_count, &op_count);
  printf("\n=== Test Completed ===\n");
  printf("Driver State: %d\n", state);
  printf("Total Operations: %lu\n", op_count);
  printf("Total Errors: %d\n", error_count);
  printf("Success Rate: %.1f%%\n", 
         op_count > 0 ? (100.0 * (op_count - error_count) / op_count) : 0.0);

  return 0;
}

/****************************************************************************
 * Name: app_logic_run_toggle_test
 *
 * Description:
 *   Test toggle functionality
 *   - Demonstrates read-modify-write operations through layers
 *
 * Input Parameters:
 *   ctx - Driver context
 *
 * Returned Value:
 *   0 on success, negative value on failure
 ****************************************************************************/

int app_logic_run_toggle_test(gpio_driver_context_t *ctx)
{
  int ret;
  int i;
  bool gpio_state;

  printf("\n=== GPIO Toggle Test ===\n");
  printf("Testing read-modify-write through layer stack\n\n");

  for (i = 0; i < 5; i++)
    {
      printf("[%d/5] ", i + 1);

      /* Read current state */
      ret = gpio_driver_execute_command(ctx, GPIO_CMD_READ, false, &gpio_state);
      if (ret != GPIO_DRIVER_OK)
        {
          printf("FAILED to read GPIO state\n");
          return -1;
        }
      printf("Current: %s -> ", gpio_state ? "HIGH" : "LOW");

      /* Toggle */
      ret = gpio_driver_execute_command(ctx, GPIO_CMD_TOGGLE, false, NULL);
      if (ret != GPIO_DRIVER_OK)
        {
          printf("FAILED to toggle GPIO\n");
          return -1;
        }

      /* Verify new state */
      ret = gpio_driver_execute_command(ctx, GPIO_CMD_READ, false, &gpio_state);
      if (ret != GPIO_DRIVER_OK)
        {
          printf("FAILED to verify GPIO state\n");
          return -1;
        }
      printf("New: %s\n", gpio_state ? "HIGH" : "LOW");

      usleep(500000); /* 500ms delay */
    }

  return 0;
}

/****************************************************************************
 * Name: app_logic_print_system_info
 *
 * Description:
 *   In thông tin hệ thống và architecture
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 ****************************************************************************/

void app_logic_print_system_info(void)
{
  printf("\n=== System Information ===\n");
  printf("Target Board: STM32F411-Minimum (BlackPill)\n");
  printf("GPIO Pin: PC13 (Built-in LED)\n");
  printf("OS: NuttX RTOS\n");
  printf("\n=== Software Architecture ===\n");
  printf("Layer 1 (User):     Application Logic\n");
  printf("Layer 2 (Driver):   GPIO Driver Layer\n");
  printf("Layer 3 (HAL):      Hardware Abstraction Layer\n");
  printf("Layer 4 (NuttX):    NuttX GPIO Subsystem\n");
  printf("Layer 5 (Kernel):   NuttX Kernel & VFS\n");
  printf("Layer 6 (Hardware): STM32F411 GPIO Registers\n");
  printf("\n=== Data Flow ===\n");
  printf("app_logic -> gpio_driver -> gpio_hal -> NuttX GPIO -> STM32 HW\n");
  printf("==========================\n\n");
}

/* Function prototypes for external use */
int app_logic_run_blink_test(gpio_driver_context_t *ctx);
int app_logic_run_toggle_test(gpio_driver_context_t *ctx);
void app_logic_print_system_info(void);

