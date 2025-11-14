/****************************************************************************
 * apps/examples/imu_system/drivers/spi/spi_manager.h
 *
 * SPI Bus Manager with mutex protection
 *
 ****************************************************************************/

#ifndef __APPS_EXAMPLES_IMU_SYSTEM_DRIVERS_SPI_SPI_MANAGER_H
#define __APPS_EXAMPLES_IMU_SYSTEM_DRIVERS_SPI_SPI_MANAGER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SPI_MAX_DEVICES 5

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: spi_manager_init
 *
 * Description:
 *   Initialize SPI manager and open SPI device
 *
 * Input Parameters:
 *   bus - SPI bus number
 *
 * Returned Value:
 *   0 on success, negative errno on failure
 *
 ****************************************************************************/

int spi_manager_init(int bus);

/****************************************************************************
 * Name: spi_manager_deinit
 *
 * Description:
 *   Deinitialize SPI manager and close SPI device
 *
 ****************************************************************************/

void spi_manager_deinit(void);

/****************************************************************************
 * Name: spi_manager_lock
 *
 * Description:
 *   Lock SPI bus for exclusive access
 *
 * Returned Value:
 *   0 on success, negative errno on failure
 *
 ****************************************************************************/

int spi_manager_lock(void);

/****************************************************************************
 * Name: spi_manager_unlock
 *
 * Description:
 *   Unlock SPI bus
 *
 ****************************************************************************/

void spi_manager_unlock(void);

/****************************************************************************
 * Name: spi_manager_transfer
 *
 * Description:
 *   Perform SPI transfer with device
 *
 * Input Parameters:
 *   devid   - Device ID (0-4)
 *   txbuf   - Transmit buffer
 *   rxbuf   - Receive buffer
 *   len     - Transfer length
 *
 * Returned Value:
 *   0 on success, negative errno on failure
 *
 ****************************************************************************/

int spi_manager_transfer(uint8_t devid, const uint8_t *txbuf,
                         uint8_t *rxbuf, size_t len);

/****************************************************************************
 * Name: spi_manager_write_reg
 *
 * Description:
 *   Write a register via SPI
 *
 * Input Parameters:
 *   devid - Device ID
 *   reg   - Register address
 *   value - Value to write
 *
 * Returned Value:
 *   0 on success, negative errno on failure
 *
 ****************************************************************************/

int spi_manager_write_reg(uint8_t devid, uint8_t reg, uint8_t value);

/****************************************************************************
 * Name: spi_manager_read_reg
 *
 * Description:
 *   Read a register via SPI
 *
 * Input Parameters:
 *   devid - Device ID
 *   reg   - Register address
 *   value - Pointer to store read value
 *
 * Returned Value:
 *   0 on success, negative errno on failure
 *
 ****************************************************************************/

int spi_manager_read_reg(uint8_t devid, uint8_t reg, uint8_t *value);

/****************************************************************************
 * Name: spi_manager_read_regs
 *
 * Description:
 *   Read multiple registers via SPI
 *
 * Input Parameters:
 *   devid  - Device ID
 *   reg    - Starting register address
 *   buffer - Buffer to store read values
 *   len    - Number of bytes to read
 *
 * Returned Value:
 *   0 on success, negative errno on failure
 *
 ****************************************************************************/

int spi_manager_read_regs(uint8_t devid, uint8_t reg,
                          uint8_t *buffer, size_t len);

#endif /* __APPS_EXAMPLES_IMU_SYSTEM_DRIVERS_SPI_SPI_MANAGER_H */
