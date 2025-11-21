#ifndef __APPS_EXAMPLES_UAV_STATES_DRIVERS_SENSORS_ICM42688P_ICM42688P_SPI_H
#define __APPS_EXAMPLES_UAV_STATES_DRIVERS_SENSORS_ICM42688P_ICM42688P_SPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <nuttx/config.h>
#include <nuttx/spi/spi.h>
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif
/****************************************************************************
 * Public Types
 ****************************************************************************/
typedef struct
{
  spi_dev_t spi;          /* SPI device structure */
  uint8_t bus;            /* SPI bus number */
  uint8_t devid;          /* Device ID / CS line */
  bool initialized;       /* Initialization flag */
  uint8_t current_bank;   /* Current register bank */
} icm42688p_dev_t;