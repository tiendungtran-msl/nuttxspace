#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <nuttx/timers/timer.h>
#include <nuttx/ioctl.h>

// Định nghĩa đường dẫn thiết bị
#define TIMER_DEVPATH   "/dev/timer1"
#define GPIO_DEVPATH    "/dev/gpio_c13"

// Biến toàn cục để theo dõi trạng thái LED
static bool led_state = false;
static int gpio_fd = -1;

/****************************************************************************
 * Timer interrupt callback function
 ****************************************************************************/
static bool timer_handler(FAR uint32_t *next_interval_us, FAR void *arg)
{
    // Đảo trạng thái LED
    led_state = !led_state;
    
    // Ghi trạng thái mới vào GPIO
    if (gpio_fd >= 0)
    {
        if (write(gpio_fd, &led_state, sizeof(bool)) < 0)
        {
            printf("ERROR: Failed to write to GPIO\n");
        }
        else
        {
            printf("LED %s\n", led_state ? "ON" : "OFF");
        }
    }
    
    // Thiết lập interval cho lần tiếp theo (1 giây = 1,000,000 microseconds)
    *next_interval_us = 1000000;
    
    // Trả về true để tiếp tục timer
    return true;
}

/****************************************************************************
 * Main function
 ****************************************************************************/
int main(int argc, FAR char *argv[])
{
    int timer_fd;
    struct timer_sethandler_s handler;
    int ret;
    
    printf("Starting Timer1 GPIO Control Application\n");
    
    // Mở thiết bị GPIO PC13
    gpio_fd = open(GPIO_DEVPATH, O_WRONLY);
    if (gpio_fd < 0)
    {
        printf("ERROR: Failed to open %s\n", GPIO_DEVPATH);
        return EXIT_FAILURE;
    }
    printf("GPIO device opened successfully\n");
    
    // Mở thiết bị Timer1
    timer_fd = open(TIMER_DEVPATH, O_RDONLY);
    if (timer_fd < 0)
    {
        printf("ERROR: Failed to open %s\n", TIMER_DEVPATH);
        close(gpio_fd);
        return EXIT_FAILURE;
    }
    printf("Timer1 device opened successfully\n");
    
    // Cấu hình timer handler
    handler.handler = timer_handler;
    handler.arg = NULL;
    
    // Thiết lập handler cho timer
    ret = ioctl(timer_fd, TCIOC_SETHANDLER, (unsigned long)&handler);
    if (ret < 0)
    {
        printf("ERROR: Failed to set timer handler\n");
        goto errout_with_dev;
    }
    printf("Timer handler set successfully\n");
    
    // Thiết lập timeout ban đầu (1 giây)
    ret = ioctl(timer_fd, TCIOC_SETTIMEOUT, 1000000); // 1,000,000 microseconds = 1 second
    if (ret < 0)
    {
        printf("ERROR: Failed to set timer timeout\n");
        goto errout_with_dev;
    }
    printf("Timer timeout set to 1 second\n");
    
    // Bắt đầu timer
    ret = ioctl(timer_fd, TCIOC_START, 0);
    if (ret < 0)
    {
        printf("ERROR: Failed to start timer\n");
        goto errout_with_dev;
    }
    printf("Timer1 started successfully\n");
    printf("Press Ctrl+C to stop the application\n");
    
    // Vòng lặp chính - chờ timer interrupt
    while (1)
    {
        // Ngủ để tiết kiệm CPU, timer interrupt sẽ thực hiện công việc
        sleep(1);
    }
    
errout_with_dev:
    // Dọn dẹp tài nguyên
    if (timer_fd >= 0)
    {
        ioctl(timer_fd, TCIOC_STOP, 0);
        close(timer_fd);
    }
    
    if (gpio_fd >= 0)
    {
        close(gpio_fd);
    }
    
    printf("Application terminated\n");
    return ret < 0 ? EXIT_FAILURE : EXIT_SUCCESS;
}