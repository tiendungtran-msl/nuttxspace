/****************************************************************************
 * apps/examples/icm42688p/icm42688p_spi.c
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <debug.h>
#include <nuttx/spi/spi.h>

#include "icm42688p_spi.h"
#include "icm42688p_registers.h"

int icm42688p_spi_init(int bus, int devid)
{
    char devpath[16];
    int fd;
    uint32_t mode = ICM42688P_SPI_MODE;
    uint32_t freq = ICM42688P_SPI_FREQUENCY;
    uint8_t bits = 8;

    snprintf(devpath, sizeof(devpath), "/dev/spi%d", gobus);
    
    fd = open(devpath, O_RDWR);
    if (fd < 0) {
        snerr("ERROR: Failed to open %s: %d\n", devpath, errno);
        return -errno;
    }

    /* Configure SPI mode */
    if (ioctl(fd, SPIOC_SETMODE, (unsigned long)mode) < 0) {
        snerr("ERROR: Failed to set SPI mode\n");
        close(fd);
        return -errno;
    }

    /* Configure SPI frequency */
    if (ioctl(fd, SPIOC_SETFREQUENCY, (unsigned long)freq) < 0) {
        snerr("ERROR: Failed to set SPI frequency\n");
        close(fd);
        return -errno;
    }

    /* Configure bits per word */
    if (ioctl(fd, SPIOC_SETBITS, (unsigned long)bits) < 0) {
        snerr("ERROR: Failed to set bits per word\n");
        close(fd);
        return -errno;
    }

    return fd;
}

void icm42688p_spi_deinit(int fd)
{
    if (fd >= 0) {
        close(fd);
    }
}

int icm42688p_write_register(int fd, uint8_t reg, uint8_t value)
{
    uint8_t txbuf[2];
    ssize_t ret;

    txbuf[0] = reg & 0x7F;  /* Clear read bit */
    txbuf[1] = value;

    ret = write(fd, txbuf, 2);
    if (ret != 2) {
        snerr("ERROR: SPI write failed: %d\n", ret);
        return -EIO;
    }

    return OK;
}

int icm42688p_read_register(int fd, uint8_t reg, uint8_t *value)
{
    uint8_t txbuf[2];
    uint8_t rxbuf[2];
    struct spi_trans_s trans;
    int ret;

    txbuf[0] = reg | SPI_READ_BIT;
    txbuf[1] = 0;

    trans.deselect = true;
    trans.cmd      = txbuf[0];
    trans.addr     = 0;
    trans.addrlen  = 0;
    trans.data     = rxbuf;
    trans.length   = 2;

    ret = ioctl(fd, SPIOC_TRANSFER, (unsigned long)&trans);
    if (ret < 0) {
        snerr("ERROR: SPI read failed\n");
        return ret;
    }

    *value = rxbuf[1];
    return OK;
}

int icm42688p_read_registers(int fd, uint8_t reg, uint8_t *buffer, size_t len)
{
    uint8_t *txbuf;
    struct spi_trans_s trans;
    int ret;

    txbuf = malloc(len + 1);
    if (!txbuf) {
        return -ENOMEM;
    }

    txbuf[0] = reg | SPI_READ_BIT;
    memset(&txbuf[1], 0, len);

    trans.deselect = true;
    trans.cmd      = txbuf[0];
    trans.addr     = 0;
    trans.addrlen  = 0;
    trans.data     = buffer;
    trans.length   = len + 1;

    ret = ioctl(fd, SPIOC_TRANSFER, (unsigned long)&trans);
    
    free(txbuf);
    
    if (ret < 0) {
        snerr("ERROR: SPI burst read failed\n");
        return ret;
    }

    return OK;
}

int icm42688p_select_bank(int fd, uint8_t bank)
{
    return icm42688p_write_register(fd, ICM42688P_REG_BANK_SEL, bank);
}