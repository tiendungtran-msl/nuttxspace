/****************************************************************************
 * apps/examples/imu_system/drivers/led/led_driver.c
 *
 * LED Driver Implementation
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <debug.h>
#include <nuttx/clock.h>

#include "led_driver.h"
#include "../../utils/config.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LED_DEVPATH "/dev/gpout0"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static int g_led_fd = -1;
static bool g_led_state = false;
static uint32_t g_blink_period_ms = 0;
static uint64_t g_last_toggle_time = 0;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: led_init
 ****************************************************************************/

int led_init(void)
{
  g_led_fd = open(LED_DEVPATH, O_WRONLY);
  if (g_led_fd < 0)
    {
      /* LED device might not be available, that's OK */

      sninfo("LED device not available at %s, using dummy mode\n",
             LED_DEVPATH);
      g_led_fd = -1;
      return OK;
    }

  sninfo("LED driver initialized\n");

  /* Turn LED off initially */

  led_set(false);

  return OK;
}

/****************************************************************************
 * Name: led_deinit
 ****************************************************************************/

void led_deinit(void)
{
  if (g_led_fd >= 0)
    {
      led_set(false);
      close(g_led_fd);
      g_led_fd = -1;
    }
}

/****************************************************************************
 * Name: led_set
 ****************************************************************************/

void led_set(bool on)
{
  char buf[2];

  g_led_state = on;
  g_blink_period_ms = 0;  /* Stop blinking */

  if (g_led_fd >= 0)
    {
      buf[0] = on ? '1' : '0';
      buf[1] = '\0';
      write(g_led_fd, buf, 1);
    }
}

/****************************************************************************
 * Name: led_blink
 ****************************************************************************/

void led_blink(uint32_t period_ms)
{
  g_blink_period_ms = period_ms;
  g_last_toggle_time = clock_systime_ticks() * (1000 / CLK_TCK);

  if (period_ms == 0)
    {
      led_set(false);
    }
}

/****************************************************************************
 * Name: led_update
 ****************************************************************************/

void led_update(void)
{
  uint64_t current_time;
  uint64_t elapsed_ms;

  if (g_blink_period_ms == 0)
    {
      return;
    }

  current_time = clock_systime_ticks() * (1000 / CLK_TCK);
  elapsed_ms = current_time - g_last_toggle_time;

  if (elapsed_ms >= g_blink_period_ms / 2)
    {
      g_led_state = !g_led_state;
      g_last_toggle_time = current_time;

      if (g_led_fd >= 0)
        {
          char buf[2];
          buf[0] = g_led_state ? '1' : '0';
          buf[1] = '\0';
          write(g_led_fd, buf, 1);
        }
    }
}
