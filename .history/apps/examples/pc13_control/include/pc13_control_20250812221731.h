/****************************************************************************
 * include/pc13_control.h
 ****************************************************************************/

#ifndef __PC13_CONTROL_H
#define __PC13_CONTROL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Application states */
typedef enum
{
  PC13_STATE_IDLE = 0,
  PC13_STATE_MANUAL,
  PC13_STATE_AUTO_BLINK,
  PC13_STATE_ERROR
} pc13_state_t;

/* Application context */
typedef struct
{
  pc13_state_t state;
  bool pin_state;
  uint32_t blink_interval;
  bool running;
} pc13_context_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Main application functions */
int pc13_control_init(void);
int pc13_control_deinit(void);
void pc13_control_main_loop(void);

/* Command processing */
int process_command(const char *cmd);
void print_help(void);
void print_status(void);

/* Utility functions */
void delay_ms(uint32_t ms);
uint32_t get_timestamp(void);

#endif /* __PC13_CONTROL_H */