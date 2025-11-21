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

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Cấu hình các chân CS cho các cảm biến ICM42688P */

#define GPIO_SPI1_CS_ICM0 (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                           GPIO_OUTPUT_SET | GPIO_PORTA | GPIO_PIN4)
#define GPIO_SPI1_CS_ICM1 (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                           GPIO_OUTPUT_SET | GPIO_PORTD | GPIO_PIN11)
#define GPIO_SPI1_CS_ICM2 (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                           GPIO_OUTPUT_SET | GPIO_PORTD | GPIO_PIN12)
#define GPIO_SPI1_CS_ICM3 (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                           GPIO_OUTPUT_SET | GPIO_PORTD | GPIO_PIN13)

/* SPI Configuration for ICM42688P */

#define ICM42688P_SPI_FREQUENCY     10000000  /* 10 MHz */
#define ICM42688P_SPI_MODE          SPIDEV_MODE3

#define NUM_ICM42688P               4

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct icm42688p_spi_wrapper_s
{
  struct spi_dev_s spidev;      /* Must be first */
  struct spi_dev_s *parent;     /* Parent SPI device */
  uint8_t sensor_id;            /* Sensor ID (0-3) */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* SPI1 device handle */

static struct spi_dev_s *g_spi1_icm = NULL;

/* CS GPIO pins array */

static const uint32_t g_icm_cs_pins[NUM_ICM42688P] =
{
  GPIO_SPI1_CS_ICM0,
  GPIO_SPI1_CS_ICM1,
  GPIO_SPI1_CS_ICM2,
  GPIO_SPI1_CS_ICM3
};

/* Khai báo thiết bị ICM */
void stm32_spidev_icm_initialize(void);

/* Tạo kết nối SPI */
int board_spi1_icm_initialize(void);

/* Lựa chọn cảm biến ICM thông qua chân CS */
void board_spi1_icm_select(uint8_t id, bool selected);

struct spi_dev_s *board_spi1_icm_get_handle(void);

struct spi_dev_s *board_spi1_icm_get_device(uint8_t id);

#ifdef __cplusplus
}
#endif

#endif /* __BOARDS_ARM_STM32H7_YOUR_BOARD_SRC_STM32_SPI_ICM_H */