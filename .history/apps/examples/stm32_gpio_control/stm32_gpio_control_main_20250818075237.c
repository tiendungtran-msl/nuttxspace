## File: stm32_gpio_control_main.c
```c
/****************************************************************************
 * examples/stm32_gpio_control/stm32_gpio_control_main.c
 *
 * Main Application Entry Point
 * Điểm khởi đầu ứng dụng - Tích hợp tất cả các layers
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "include/gpio_driver.h"
#include "include/app_config.h"

/* External function prototypes from app_logic.c */
extern int app_logic_run_blink_test(gpio_driver_context_t *ctx);
extern int app_logic_run_toggle_test(gpio_driver_context_t *ctx);
extern void app_logic_print_system_info(void);

/* Board GPIO setup functions */
extern int board_gpio_setup(void);
extern int board_gpio_cleanup(void);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: print_usage
 *
 * Description:
 *   In hướng dẫn sử dụng ứng dụng
 *
 * Input Parameters:
 *   progname - Program name
 ****************************************************************************/

static void print_usage(const char *progname)
{
  printf("\nUsage: %s [options]\n", progname);
  printf("Options:\n");
  printf("  -h, --help     Show this help message\n");
  printf("  -i, --info     Show system information\n");
  printf("  -b, --blink    Run LED blink test\n");
  printf("  -t, --toggle   Run GPIO toggle test\n");
  printf("  -a, --all      Run all tests\n");
  printf("\nDefault: Run blink test if no options specified\n");
  printf("\nExample:\n");
  printf("  %s -b          # Run blink test\n", progname);
  printf("  %s --all       # Run all tests\n", progname);
  printf("\n");
}

/****************************************************************************
 * Name: parse_arguments
 *
 * Description:
 *   Phân tích command line arguments
 *
 * Input Parameters:
 *   argc - Argument count
 *   argv - Argument vector
 *   show_info - Pointer to store info flag
 *   run_blink - Pointer to store blink flag  
 *   run_toggle - Pointer to store toggle flag
 *
 * Returned Value:
 *   0 on success, -1 on error or help requested
 ****************************************************************************/

static int parse_arguments(int argc, char *argv[], 
                          bool *show_info, 
                          bool *run_blink, 
                          bool *run_toggle)
{
  int i;

  /* Initialize flags */
  *show_info = false;
  *run_blink = false;
  *run_toggle = false;

  /* If no arguments, run default blink test */
  if (argc == 1)
    {
      *run_blink = true;
      return 0;
    }

  /* Parse arguments */
  for (i = 1; i < argc; i++)
    {
      if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0)
        {
          print_usage(argv[0]);
          return -1;
        }
      else if (strcmp(argv[i], "-i") == 0 || strcmp(argv[i], "--info") == 0)
        {
          *show_info = true;
        }
      else if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--blink") == 0)
        {
          *run_blink = true;
        }
      else if (strcmp(argv[i], "-t") == 0 || strcmp(argv[i], "--toggle") == 0)
        {
          *run_toggle = true;
        }
      else if (strcmp(argv[i], "-a") == 0 || strcmp(argv[i], "--all") == 0)
        {
          *show_info = true;
          *run_blink = true;
          *run_toggle = true;
        }
      else
        {
          printf("Unknown option: %s\n", argv[i]);
          print_usage(argv[0]);
          return -1;
        }
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: main
 *
 * Description:
 *   Main entry point for STM32 GPIO Control Example
 *   
 *   Architecture Overview:
 *   1. User Layer (main) - Command parsing and test orchestration
 *   2. Application Logic Layer - Business logic and test implementations  
 *   3. Driver Layer - GPIO operations and state management
 *   4. HAL Layer - Hardware abstraction and NuttX interface
 *   5. NuttX GPIO Subsystem - OS-level GPIO management
 *   6. STM32 Hardware - Physical GPIO registers and pins
 *
 * Input Parameters:
 *   argc - Argument count
 *   argv - Argument vector
 *
 * Returned Value:
 *   EXIT_SUCCESS on success, EXIT_FAILURE on failure
 ****************************************************************************/

int main(int argc, char *argv[])
{
  gpio_driver_context_t gpio_ctx;
  bool show_info, run_blink, run_toggle;
  int ret;
  app_state_t final_state;
  int error_count;
  uint32_t op_count;

  printf("\n");
  printf("**********************************************************\n");
  printf("*       NuttX STM32 GPIO Control Example                *\n");
  printf("*       STM32F411 BlackPill - PC13 LED Control          *\n");
  printf("*       Layered Architecture Demonstration              *\n");
  printf("**********************************************************\n");

  /* Setup GPIO devices if not done at board level */
  printf("=== Setting up GPIO devices ===\n");
  ret = board_gpio_setup();
  if (ret != 0)
    {
      printf("ERROR: Failed to setup GPIO devices: %d\n", ret);
      printf("Make sure GPIO driver is enabled in kernel config\n");
      return EXIT_FAILURE;
    }
  printf("GPIO devices created successfully\n");

  /* Parse command line arguments */
  ret = parse_arguments(argc, argv, &show_info, &run_blink, &run_toggle);
  if (ret != 0)
    {
      return (ret == -1 && argc > 1) ? EXIT_SUCCESS : EXIT_FAILURE;
    }

  /* Show system information if requested */
  if (show_info)
    {
      app_logic_print_system_info();
    }

  /* Initialize GPIO driver (Driver Layer -> HAL Layer -> NuttX) */
  printf("=== Initializing GPIO Driver Stack ===\n");
  printf("Step 1: Driver Layer initialization...\n");
  ret = gpio_driver_init(&gpio_ctx);
  if (ret != GPIO_DRIVER_OK)
    {
      printf("ERROR: Failed to initialize GPIO driver\n");
      printf("Check if GPIO device is available and permissions are correct\n");
      return EXIT_FAILURE;
    }
  printf("Step 2: HAL Layer connected to NuttX GPIO subsystem\n");
  printf("Step 3: NuttX GPIO subsystem ready\n");
  printf("Step 4: STM32F411 PC13 hardware configured\n");
  printf("GPIO Driver Stack initialized successfully!\n");

  /* Run tests based on user selection */
  if (run_blink)
    {
      ret = app_logic_run_blink_test(&gpio_ctx);
      if (ret != 0)
        {
          printf("ERROR: Blink test failed\n");
          goto cleanup;
        }
    }

  if (run_toggle)
    {
      ret = app_logic_run_toggle_test(&gpio_ctx);
      if (ret != 0)
        {
          printf("ERROR: Toggle test failed\n");
          goto cleanup;
        }
    }

  /* Success path */
  ret = EXIT_SUCCESS;

cleanup:
  /* Cleanup and final status */
  printf("\n=== Cleaning Up GPIO Driver Stack ===\n");
  
  /* Get final statistics before cleanup */
  gpio_driver_get_status(&gpio_ctx, &final_state, &error_count, &op_count);
  
  /* Cleanup driver resources */
  gpio_driver_cleanup(&gpio_ctx);

  /* Cleanup board GPIO devices */
  board_gpio_cleanup();
  printf("GPIO devices cleaned up\n");
  
  printf("Final Statistics:\n");
  printf("  Driver State: %s\n", 
         (final_state == APP_STATE_RUNNING) ? "RUNNING" :
         (final_state == APP_STATE_STOPPED) ? "STOPPED" :
         (final_state == APP_STATE_ERROR) ? "ERROR" : "INIT");
  printf("  Total Operations: %lu\n", op_count);
  printf("  Total Errors: %d\n", error_count);
  printf("  Success Rate: %.1f%%\n", 
         op_count > 0 ? (100.0 * (op_count - error_count) / op_count) : 0.0);

  printf("\n");
  printf("**********************************************************\n");
  printf("*               Application Completed                   *\n");
  printf("*   All GPIO driver stack layers properly cleaned up    *\n");
  printf("**********************************************************\n");

  return ret;
}