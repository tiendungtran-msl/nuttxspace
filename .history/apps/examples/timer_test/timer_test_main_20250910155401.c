#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <nuttx/timers/timer.h>
#include <errno.h>

#define TIMER_DEVICE "/dev/timer1"
#define GPIO_DEVICE "/dev/gpio_c13"
#define TIMER_INTERVAL_MS 1000

static int gpio_fd = -1;
static bool gpio_state = false;

static void timer_handler(int signo, siginfo_t *info, void *context)
{
    static int count = 0;
    
    /* Toggle GPIO state */
    gpio_state = !gpio_state;
    
    /* Write to GPIO */
    if (gpio_fd >= 0) {
        char value = gpio_state ? '1' : '0';
        write(gpio_fd, &value, 1);
        printf("Timer interrupt %d: GPIO C13 = %c\n", ++count, value);
    }
}

int main(int argc, FAR char *argv[])
{
    int timer_fd;
    struct timer_sethandler_s handler;
    uint32_t timeout_us;
    int ret;
    
    printf("Timer Test Application for STM32F411\n");
    printf("Blinking GPIO C13 every %d ms using timer1\n", TIMER_INTERVAL_MS);
    
    /* Open timer device */
    timer_fd = open(TIMER_DEVICE, O_RDONLY);
    if (timer_fd < 0) {
        printf("ERROR: Failed to open %s: %d\n", TIMER_DEVICE, errno);
        return EXIT_FAILURE;
    }
    
    /* Open GPIO device */
    gpio_fd = open(GPIO_DEVICE, O_WRONLY);
    if (gpio_fd < 0) {
        printf("ERROR: Failed to open %s: %d\n", GPIO_DEVICE, errno);
        close(timer_fd);
        return EXIT_FAILURE;
    }
    
    /* Set up signal handler for timer interrupt */
    handler.tch = timer_handler;
    handler.arg = NULL;
    
    ret = ioctl(timer_fd, TCIOC_SETHANDLER, (unsigned long)&handler);
    if (ret < 0) {
        printf("ERROR: Failed to set timer handler: %d\n", errno);
        goto cleanup;
    }
    
    /* Convert milliseconds to microseconds */
    timeout_us = TIMER_INTERVAL_MS * 1000;
    
    /* Set timer timeout */
    ret = ioctl(timer_fd, TCIOC_SETTIMEOUT, timeout_us);
    if (ret < 0) {
        printf("ERROR: Failed to set timer timeout: %d\n", errno);
        goto cleanup;
    }
    
    /* Start the timer */
    ret = ioctl(timer_fd, TCIOC_START, 0);
    if (ret < 0) {
        printf("ERROR: Failed to start timer: %d\n", errno);
        goto cleanup;
    }
    
    printf("Timer started successfully. Press Ctrl+C to stop.\n");
    
    /* Main loop - keep the application running */
    while (1) {
        sleep(1);
    }
    
cleanup:
    /* Stop timer */
    ioctl(timer_fd, TCIOC_STOP, 0);
    
    /* Close devices */
    if (gpio_fd >= 0) {
        close(gpio_fd);
    }
    close(timer_fd);
    
    return EXIT_SUCCESS;
}