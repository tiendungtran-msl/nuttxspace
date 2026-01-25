/****************************************************************************
 * boards/arm/stm32h7/weact-stm32h743/src/stm32_spi_icm.h
 *
 * Description:
 *   SPI1 peripheral initialization header for ICM42688P IMU sensors
 *
 ****************************************************************************/

#ifndef __BOARDS_ARM_STM32H7_WEACT_STM32H743_SRC_STM32_SPI_ICM_H
#define __BOARDS_ARM_STM32H7_WEACT_STM32H743_SRC_STM32_SPI_ICM_H

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

 /* SPI Configuration for ICM42688P */

#define ICM42688P_SPI_FREQUENCY     10000000  /* 10 MHz */
#define ICM42688P_SPI_MODE          SPIDEV_MODE3

#define NUM_ICM42688P               4

#ifdef __cplusplus
extern "C"
{
#endif

/* Khai báo thiết bị ICM */
void stm32_spidev_icm_initialize(void);

/* Tạo kết nối SPI1, cấu hình thuộc tính, khai báo thiết bị wrapper */
int board_spi1_icm_initialize(void);

/* Lựa chọn cảm biến ICM thông qua chân CS */
void board_spi1_icm_select(uint8_t id, bool selected);

/* Lấy handle SPI1 chung */
struct spi_dev_s *board_spi1_icm_get_handle(void);

/* Lấy thiết bị SPI1 theo ID */
struct spi_dev_s *board_spi1_icm_get_device(uint8_t id);

#ifdef __cplusplus
}
#endif

#endif /* __BOARDS_ARM_STM32H7_YOUR_BOARD_SRC_STM32_SPI_ICM_H */