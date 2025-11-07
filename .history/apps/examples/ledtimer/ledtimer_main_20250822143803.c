#include <nuttx/config.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <nuttx/timers/timer.h>
#include <nuttx/ioexpander/gpio.h>

/* File descriptor */
static int timer_fd;
static int gpio_fd;

/* Hàm toggle LED PC13 */
static void toggle_led(void)
{
  static bool state = false;
  state = !state;

  if (state)
    {
      ioctl(gpio_fd, GPIOC_SET, 1);   /* Bật LED */
    }
  else
    {
      ioctl(gpio_fd, GPIOC_SET, 0);   /* Tắt LED */
    }
}

/* Hàm main */
int main(int argc, FAR char *argv[])
{
  int ret;
  struct timer_status_s ts;

  printf("LED Timer App started!\n");

  /* Mở GPIO C13 (được build thành /dev/gpio_c13) */
  gpio_fd = open("/dev/gpio_c13", O_RDWR);
  if (gpio_fd < 0)
    {
      perror("open gpio_c13 failed");
      return -1;
    }

  /* Mở Timer0 */
  timer_fd = open("/dev/timer0", O_RDONLY);
  if (timer_fd < 0)
    {
      perror("open timer0 failed");
      return -1;
    }

  /* Cấu hình timer 1Hz (chu kỳ 1 giây) */
  struct timer_settimeout_s to;
  to.flags = 0;          /* periodic mode */
  to.ns = 1000000000ULL; /* 1s = 1e9 ns */

  ret = ioctl(timer_fd, TCIOC_SETTIMEOUT, (unsigned long)&to);
  if (ret < 0)
    {
      perror("ioctl SETTIMEOUT failed");
      return -1;
    }

  /* Start timer */
  ret = ioctl(timer_fd, TCIOC_START, 0);
  if (ret < 0)
    {
      perror("ioctl START failed");
      return -1;
    }

  printf("Timer started, LED will blink every 1s!\n");

  /* Vòng lặp: mỗi khi timer hết hạn thì read() sẽ unblock */
  while (1)
    {
      uint64_t expirations;
g
      ret = read(timer_fd, &expirations, sizeof(expirations));
      if (ret > 0)
        {
          toggle_led();
        }
    }

  close(timer_fd);
  close(gpio_fd);

  return 0;
}
