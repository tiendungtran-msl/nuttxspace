/****************************************************************************
 * apps/examples/blink/blink_main.c
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <nuttx/ioexpander/gpio.h>

int main(int argc, char *argv[])
{
  int fd;
  bool led_state = false;

  printf("LED Blink - 1 second cycle\n");

  /* Mở GPIO device */
  fd = open("/dev/gpio0", O_RDWR);
  if (fd < 0)
    {
      printf("ERROR: Cannot open /dev/gpio0\n");
      return -1;
    }

  printf("Press Ctrl+C to stop\n\n");

  /* Vòng lặp nhấp nháy */
  while (1)
    {
      /* Đổi trạng thái */
      led_state = !led_state;

      /* Ghi vào GPIO */
      ioctl(fd, GPIOC_WRITE, (unsigned long)&led_state);

      /* In trạng thái */
      printf("LED: %s\n", led_state ? "ON " : "OFF");

      /* Đợi 500ms */
      usleep(500000);
    }

  close(fd);
  return 0;
}