#ifndef __INCLUDE_NUTTX_SPI_SPI_ICM42688P_H
#define __INCLUDE_NUTTX_SPI_SPI_ICM42688P_H

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <nuttx/config.h>
#include <nuttx/spi/spi.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>
#include <string.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <nuttx/spi/spi_transfer.h>
#include <syslog.h>
#include <stdlib.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

 struct icm42688p_dev_s
 {
    spi_dev_s spi;               /* SPI device handle */
    uint8_t id;                 /* Device ID */
    bool initialized;           /* Initialization flag */
 };

 struct icm42688p_data_s
 {
    uint8_t bank;   /* Lựa chọn bank */
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float temperature;
 };
/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

 int icm42688p_init(struct icm42688p_dev_s *dev, uint8_t bus, uint8_t devid);

 int 
#ifdef __cplusplus
extern "C"
{
#endif

#ifdef __cplusplus
}
#endif