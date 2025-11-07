#include <nuttx/config.h>
#include <nuttx/fs/fs.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>

int main(int argc, FAR char *argv[])
{
  int fd;
  int i;
  bool val = false;

  fd = open("/dev/gpio0", O_RDWR);
  if (fd < 0)
    {
      printf("Failed to open /dev/gpio0\n");
      return -1;
    }

  for (i = 0; i < 10; i++)
    {
      val = !val;
      write(fd, &val, sizeof(val));
      printf("LED PC13 -> %d\n", val);
      sleep(1);
    }

  close(fd);
  return 0;
}
