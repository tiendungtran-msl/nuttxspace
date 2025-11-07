// apps/data_logger/src/hardware.c

#include <nuttx/config.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#include "tasks.h"

/* Hardware file descriptors */
static int g_i2c_fd = -1;
static int g_spi_fd = -1;
static int g_adc_fd = -1;

int hardware_initialize(void)
{
    printf("Hardware: Initializing peripherals...\n");
    
    /* Open I2C device */
    g_i2c_fd = open("/dev/i2c0", O_RDWR);
    if (g_i2c_fd < 0) {
        fprintf(stderr, "WARNING: Failed to open I2C device\n");
    } else {
        printf("Hardware: I2C opened\n");
    }
    
    /* Open SPI device */
    g_spi_fd = open("/dev/spi0", O_RDWR);
    if (g_spi_fd < 0) {
        fprintf(stderr, "WARNING: Failed to open SPI device\n");
    } else {
        printf("Hardware: SPI opened\n");
    }
    
    /* Open ADC device */
    g_adc_fd = open("/dev/adc0", O_RDONLY);
    if (g_adc_fd < 0) {
        fprintf(stderr, "WARNING: Failed to open ADC device\n");
    } else {
        printf("Hardware: ADC opened\n");
    }
    
    /* TODO: Configure peripherals (baud rates, modes, etc.) */
    
    printf("Hardware: Initialization complete\n");
    return OK;
}

void hardware_cleanup(void)
{
    printf("Hardware: Closing peripherals...\n");
    
    if (g_i2c_fd >= 0) {
        close(g_i2c_fd);
        g_i2c_fd = -1;
    }
    
    if (g_spi_fd >= 0) {
        close(g_spi_fd);
        g_spi_fd = -1;
    }
    
    if (g_adc_fd >= 0) {
        close(g_adc_fd);
        g_adc_fd = -1;
    }
    
    printf("Hardware: Cleanup complete\n");
}
