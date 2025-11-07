/****************************************************************************
 * src/commands.c
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "config.h"
#include "pc13_control.h"
#include "gpio_driver.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

extern pc13_context_t g_pc13_ctx;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int parse_args(const char *cmd, char **argv, int max_args)
{
  int argc = 0;
  char *token;
  static char cmd_copy[CMD_BUFFER_SIZE];
  
  strncpy(cmd_copy, cmd, sizeof(cmd_copy) - 1);
  cmd_copy[sizeof(cmd_copy) - 1] = '\0';
  
  token = strtok(cmd_copy, " \t");
  while (token != NULL && argc < max_args)
    {
      argv[argc++] = token;
      token = strtok(NULL, " \t");
    }
  
  return argc;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int process_command(const char *cmd)
{
  char *argv[MAX_ARGS];
  int argc;
  int ret = PC13_OK;

  argc = parse_args(cmd, argv, MAX_ARGS);
  if (argc == 0)
    {
      return PC13_OK;
    }

  if (strcmp(argv[0], "help") == 0 || strcmp(argv[0], "?") == 0)
    {
      print_help();
    }
  else if (strcmp(argv[0], "status") == 0)
    {
      print_status();
    }
  else if (strcmp(argv[0], "on") == 0)
    {
      ret = gpio_pc13_set(true);
      if (ret == PC13_OK)
        {
          g_pc13_ctx.pin_state = true;
          g_pc13_ctx.state = PC13_STATE_MANUAL;
          printf("PC13 turned ON\n");
        }
    }
  else if (strcmp(argv[0], "off") == 0)
    {
      ret = gpio_pc13_set(false);
      if (ret == PC13_OK)
        {
          g_pc13_ctx.pin_state = false;
          g_pc13_ctx.state = PC13_STATE_MANUAL;
          printf("PC13 turned OFF\n");
        }
    }
  else if (strcmp(argv[0], "toggle") == 0)
    {
      ret = gpio_pc13_toggle();
      if (ret == PC13_OK)
        {
          g_pc13_ctx.pin_state = gpio_pc13_get();
          g_pc13_ctx.state = PC13_STATE_MANUAL;
          printf("PC13 toggled to %s\n", g_pc13_ctx.pin_state ? "ON" : "OFF");
        }
    }
  else if (strcmp(argv[0], "blink") == 0)
    {
      if (argc > 1)
        {
          uint32_t interval = strtoul(argv[1], NULL, 10);
          if (interval > 0)
            {
              g_pc13_ctx.blink_interval = interval;
            }
        }
      
      printf("Starting auto-blink mode (interval: %lu ms)\n", 
             (unsigned long)g_pc13_ctx.blink_interval);
      printf("Press any key to stop blinking\n");
      g_pc13_ctx.state = PC13_STATE_AUTO_BLINK;
    }
  else if (strcmp(argv[0], "interval") == 0)
    {
      if (argc > 1)
        {
          uint32_t interval = strtoul(argv[1], NULL, 10);
          if (interval > 0)
            {
              g_pc13_ctx.blink_interval = interval;
              printf("Blink interval set to %lu ms\n", (unsigned long)interval);
            }
          else
            {
              printf("ERROR: Invalid interval value\n");
              ret = PC13_INVALID_ARG;
            }
        }
      else
        {
          printf("Current blink interval: %lu ms\n", 
                 (unsigned long)g_pc13_ctx.blink_interval);
        }
    }
  else if (strcmp(argv[0], "exit") == 0 || strcmp(argv[0], "quit") == 0)
    {
      printf("Exiting application...\n");
      g_pc13_ctx.running = false;
    }
  else
    {
      printf("Unknown command: %s\n", argv[0]);
      printf("Type 'help' for available commands\n");
      ret = PC13_INVALID_ARG;
    }

  return ret;
}

void print_help(void)
{
  printf("\nAvailable commands:\n");
  printf("  help, ?           - Show this help message\n");
  printf("  status            - Show current status\n");
  printf("  on                - Turn PC13 ON\n");
  printf("  off               - Turn PC13 OFF\n");
  printf("  toggle            - Toggle PC13 state\n");
  printf("  blink [interval]  - Start auto-blink mode\n");
  printf("  interval [ms]     - Set/get blink interval\n");
  printf("  exit, quit        - Exit application\n");
  printf("\nExamples:\n");
  printf("  blink 500         - Blink with 500ms interval\n");
  printf("  interval 2000     - Set interval to 2 seconds\n");
  printf("\n");
}

void print_status(void)
{
  const char *state_str[] = {
    "IDLE", "MANUAL", "AUTO_BLINK", "ERROR"
  };

  printf("\n--- PC13 Control Status ---\n");
  printf("State:         %s\n", state_str[g_pc13_ctx.state]);
  printf("Pin State:     %s\n", g_pc13_ctx.pin_state ? "HIGH" : "LOW");
  printf("Blink Interval: %lu ms\n", (unsigned long)g_pc13_ctx.blink_interval);
  printf("Running:       %s\n", g_pc13_ctx.running ? "YES" : "NO");
  printf("Timestamp:     %lu\n", (unsigned long)get_timestamp());
  printf("-------------------------\n\n");
}