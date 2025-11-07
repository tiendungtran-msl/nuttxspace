#include <nuttx/config.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <nuttx/ioexpander/gpio.h>

int main(int argc, FAR char *argv[])
{
    int fd;
    int value;

    /* Mở thiết bị kernel tạo ra: /dev/gpio0 */
    fd = open("/dev/gpio0", O_RDWR);
    if (fd < 0)
      {
        perror("open");
        return -1;
      }

    /* Bật LED */
    value = 1;
    ioctl(fd, GPIOC_WRITE, (unsigned long)&value);
    sleep(1);

    /* Tắt LED */
    value = 0;
    ioctl(fd, GPIOC_WRITE, (unsigned long)&value);
    sleep(1);

    close(fd);
    return 0;
}
