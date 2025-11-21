/****************************************************************************
 * apps/examples/uav_states/drivers/comm/spi/spi.h
 ****************************************************************************/

#ifndef __APPS_EXAMPLES_UAV_STATES_DRIVERS_COMM_SPI_SPI_H
#define __APPS_EXAMPLES_UAV_STATES_DRIVERS_COMM_SPI_SPI_H
#include <stdint.h>

#define SPI_READ_BIT    0x80

int icm42688p_spi_init(int bus, int devid);
void icm42688p_spi_deinit(int fd);
void icm42688p_cs_select(int devid, bool selected);
int icm42688p_write_register(int fd, int devid, uint8_t reg, uint8_t value);
int icm42688p_read_register(int fd, int devid, uint8_t reg, uint8_t *value);
int icm42688p_read_registers(int fd, int devid, uint8_t reg, uint8_t *buffer, size_t len);
int icm42688p_select_bank(int fd, int devid, uint8_t bank);

#endif /* __APPS_EXAMPLES_UAV_STATES_DRIVERS_COMM_SPI_SPI_H */