#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <signal.h>
#include <stdbool.h>

#define GPIO_DEVICE "/dev/gpio_c13"
#define BLINK_INTERVAL_MS 1000

static volatile bool running = true;

static void signal_handler(int signo)
{
    running = false;
    printf("\nStopping GPIO blink test...\n");
}

int main(int argc, FAR char *argv[])
{
    int gpio_fd;
    bool gpio_state = false;
    int count = 0;
    int ret;
    
    printf("GPIO Blink Test Application for STM32F411\n");
    printf("Blinking GPIO C13 every %d ms\n", BLINK_INTERVAL_MS);
    
    /* Set up signal handler for Ctrl+C */
    signal(SIGINT, signal_handler);
    
    /* Open GPIO device */
    gpio_fd = open(GPIO_DEVICE, O_WRONLY);
    if (gpio_fd < 0) {
        printf("ERROR: Failed to open %s: %d\n", GPIO_DEVICE, errno);
        return EXIT_FAILURE;
    }
    
    printf("GPIO device opened successfully. Press Ctrl+C to stop.\n");
    
    /* Main blink loop */
    while (running) {
        /* Toggle GPIO state */
        gpio_state = !gpio_state;
        
        /* Write to GPIO */
        char value = gpio_state ? '1' : '0';
        ret = write(gpio_fd, &value, 1);
        if (ret < 0) {
            printf("ERROR: Failed to write to GPIO: %d\n", errno);
            break;
        }
        
        printf("Blink %d: GPIO C13 = %c\n", ++count, value);
        
        /* Sleep for the specified interval */
        usleep(BLINK_INTERVAL_MS * 1000); /* Convert ms to microseconds */
    }
    
    /* Turn off GPIO before exit */
    char off = '0';
    write(gpio_fd, &off, 1);
    
    close(gpio_fd);
    printf("GPIO blink test finished.\n");
    
    return EXIT_SUCCESS;
}