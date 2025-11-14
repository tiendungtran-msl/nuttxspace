/****************************************************************************
 * apps/examples/imu_system/drivers/i2c/i2c_manager.h
 ****************************************************************************/

#ifndef __APPS_EXAMPLES_IMU_SYSTEM_DRIVERS_I2C_I2C_MANAGER_H
#define __APPS_EXAMPLES_IMU_SYSTEM_DRIVERS_I2C_I2C_MANAGER_H

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Initialize I2C manager */
int i2c_manager_init(int bus);

/* Deinitialize I2C manager */
void i2c_manager_deinit(void);

/* Read single register */
int i2c_manager_read_reg(uint8_t addr, uint8_t reg, uint8_t *value);

/* Read multiple registers */
int i2c_manager_read_regs(uint8_t addr, uint8_t reg, 
                          uint8_t *buffer, size_t len);

/* Write single register */
int i2c_manager_write_reg(uint8_t addr, uint8_t reg, uint8_t value);

/* Write multiple registers */
int i2c_manager_write_regs(uint8_t addr, uint8_t reg,
                           const uint8_t *buffer, size_t len);

#endif /* __APPS_EXAMPLES_IMU_SYSTEM_DRIVERS_I2C_I2C_MANAGER_H */