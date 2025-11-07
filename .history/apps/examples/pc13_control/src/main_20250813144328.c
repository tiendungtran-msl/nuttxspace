/****************************************************************************
 * src/main.c
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

#include "../include/config.h"
#include "../include/pc13_control.h"
#include "../include/gpio_driver.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Global application context */
pc13_context_t g_pc13_ctx =
{
  .state = PC13_STATE_IDLE,
  .pin_state = false,
  .blink_interval = DEFAULT_BLINK_INTERVAL,
  .running = false
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void print_banner(void)
{
  printf("\n");
  printf("===========================================\n");
  printf("    STM32F411 PC13 Control Application    \n");
  printf("===========================================\n");
  printf("Build: %s %s\n", __DATE__, __TIME__);
  printf("NuttX GPIO Control for BlackPill PC13\n");
  printf("Type 'help' for available commands\n");
  printf("\n");
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int pc13_control_init(void)
{
  int ret;

  printf("Initializing PC13 control...\n");

  /* Initialize GPIO */
  ret = gpio_pc13_init();
  if (ret < 0)
    {
      printf("ERROR: Failed to initialize PC13 GPIO: %d\n", ret);
      return ret;
    }

  /* Set initial state */
  g_pc13_ctx.state = PC13_STATE_IDLE;
  g_pc13_ctx.running = true;
  
  /* Turn off LED initially */
  gpio_pc13_set(false);
  g_pc13_ctx.pin_state = false;

  printf("PC13 control initialized successfully\n");
  return PC13_OK;
}

int pc13_control_deinit(void)
{
  printf("Deinitializing PC13 control...\n");
  
  g_pc13_ctx.running = false;
  g_pc13_ctx.state = PC13_STATE_IDLE;
  
  /* Turn off LED */
  gpio_pc13_set(false);
  
  gpio_pc13_deinit();
  
  printf("PC13 control deinitialized\n");
  return PC13_OK;
}

void pc13_control_main_loop(void)
{
  char cmd_buffer[CMD_BUFFER_SIZE];
  
  while (g_pc13_ctx.running)
    {
      switch (g_pc13_ctx.state)
        {
          case PC13_STATE_IDLE:
            printf("pc13> ");
            fflush(stdout);
            
            if (fgets(cmd_buffer, sizeof(cmd_buffer), stdin) != NULL)
              {
                /* Remove newline */
                cmd_buffer[strcspn(cmd_buffer, "\n")] = 0;
                
                if (strlen(cmd_buffer) > 0)
                  {
                    process_command(cmd_buffer);
                  }
              }
            break;

          case PC13_STATE_AUTO_BLINK:
            gpio_pc13_toggle();
            g_pc13_ctx.pin_state = gpio_pc13_get();
            printf("LED %s\n", g_pc13_ctx.pin_state ? "ON" : "OFF");
            delay_ms(g_pc13_ctx.blink_interval);
            
            /* Check for user input to exit blink mode */
            if (getchar() != EOF)
              {
                g_pc13_ctx.state = PC13_STATE_IDLE;
                printf("\nExiting auto-blink mode\n");
              }
            break;

          case PC13_STATE_MANUAL:
            g_pc13_ctx.state = PC13_STATE_IDLE;
            break;

          default:
            g_pc13_ctx.state = PC13_STATE_IDLE;
            break;
        }
    }
}

/****************************************************************************
 * Main Entry Point
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int ret;

  print_banner();

  /* Initialize the application */
  ret = pc13_control_init();
  if (ret < 0)
    {
      printf("Failed to initialize PC13 control application\n");
      return EXIT_FAILURE;
    }

  /* Print initial status */
  print_status();

  /* Main application loop */
  pc13_control_main_loop();

  /* Cleanup */
  pc13_control_deinit();

  printf("PC13 control application terminated\n");
  return EXIT_SUCCESS;
}