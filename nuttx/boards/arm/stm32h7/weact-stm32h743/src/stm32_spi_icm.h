/****************************************************************************
 * boards/arm/stm32h7/your-board/src/stm32_spi_icm.h
 *
 * Description:
 *   SPI1 peripheral initialization header for ICM42688P IMU sensors
 *
 ****************************************************************************/

#ifndef __BOARDS_ARM_STM32H7_YOUR_BOARD_SRC_STM32_SPI_ICM_H
#define __BOARDS_ARM_STM32H7_YOUR_BOARD_SRC_STM32_SPI_ICM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>
#include <nuttx/spi/spi.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: stm32_spidev_icm_initialize
 *
 * Description:
 *   Initialize CS GPIO pins for ICM42688P sensors
 *   This is called from stm32_spidev_initialize()
 *
 ****************************************************************************/

void stm32_spidev_icm_initialize(void);

/****************************************************************************
 * Name: board_spi1_icm_initialize
 *
 * Description:
 *   Initialize SPI1 bus for ICM42688P communication
 *
 * Returned Value:
 *   OK on success; A negated errno value on failure
 *
 ****************************************************************************/

int board_spi1_icm_initialize(void);

/****************************************************************************
 * Name: board_spi1_icm_select
 *
 * Description:
 *   Select/deselect ICM42688P chip select
 *
 * Input Parameters:
 *   id       - Sensor ID (0-3)
 *   selected - true: assert CS (LOW), false: deassert CS (HIGH)
 *
 ****************************************************************************/

void board_spi1_icm_select(uint8_t id, bool selected);

/****************************************************************************
 * Name: board_spi1_sensor_select
 *
 * Description:
 *   Select/deselect sensor chip select (ICM42688P or BMM150)
 *
 * Input Parameters:
 *   id       - Sensor ID (0-3: ICM42688P, 4: BMM150)
 *   selected - true: assert CS (LOW), false: deassert CS (HIGH)
 *
 ****************************************************************************/

void board_spi1_sensor_select(uint8_t id, bool selected);

/****************************************************************************
 * Name: board_spi1_icm_get_handle
 *
 * Description:
 *   Get SPI1 device handle for ICM42688P
 *
 * Returned Value:
 *   Pointer to SPI device structure, or NULL if not initialized
 *
 ****************************************************************************/

struct spi_dev_s *board_spi1_icm_get_handle(void);

struct spi_dev_s *board_spi1_icm_get_device(uint8_t id);

#ifdef __cplusplus
}
#endif

#endif /* __BOARDS_ARM_STM32H7_YOUR_BOARD_SRC_STM32_SPI_ICM_H */