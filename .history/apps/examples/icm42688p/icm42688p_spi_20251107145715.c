/****************************************************************************
 * apps/examples/icm42688p/icm42688p_spi.c
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <debug.h>
#include <string.h>
#include <stdlib.h>
#include <nuttx/spi/spi.h>
#include <nuttx/spi/spi_transfer.h>

#include "icm42688p_spi.h"
#include "icm42688p_registers.h"
#include "icm42688p_driver.h"

/* External function from board code */
extern void board_spi1_icm_select(uint8_t id, bool selected);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int icm42688p_spi_init(int bus, int devid)
{
    char devpath[32];
    int fd;

    /* Open /dev/spi1 */
    snprintf(devpath, sizeof(devpath), "/dev/spi%d", bus);
    
    fd = open(devpath, O_RDWR);
    if (fd < 0)
    {
        snerr("ERROR: Failed to open %s: %d\n", devpath, errno);
        return -errno;
    }

    sninfo("SPI device %s opened (fd=%d) for sensor %d\n", 
           devpath, fd, devid);
    
    return fd;
}

void icm42688p_spi_deinit(int fd)
{
    if (fd >= 0)
    {
        close(fd);
    }
}

void icm42688p_cs_select(int devid, bool selected)
{
    board_spi1_icm_select(devid, selected);
}

int icm42688p_write_register(int fd, int devid, uint8_t reg, uint8_t value)
{
    int ret;
    uint8_t tx[2];
    uint8_t rx[2];

    memset(rx, 0, sizeof(rx));
    tx[0] = reg & 0x7F;  /* Clear read bit for write */
    tx[1] = value;

    /* Manual CS control */
    icm42688p_cs_select(devid, true);
    usleep(1);  /* Small delay */

    struct spi_trans_s trans =
    {
        .deselect = false,  /* Manual CS control */
        .delay    = 0,
        .nwords   = 2,
        .txbuffer = tx,
        .rxbuffer = rx
    };

    struct spi_sequence_s seq =
    {
        .ntrans = 1,
        .trans  = &trans
    };

    ret = ioctl(fd, SPIIOC_TRANSFER, (unsigned long)&seq);
    
    usleep(1);  /* Small delay */
    icm42688p_cs_select(devid, false);

    if (ret < 0)
    {
        snerr("ERROR: SPI write failed: %s\n", strerror(errno));
        return -errno;
    }

    return OK;
}

int icm42688p_read_register(int fd, int devid, uint8_t reg, uint8_t *value)
{
    int ret;
    uint8_t tx[2];
    uint8_t rx[2];

    if (!value)
    {
        return -EINVAL;
    }

    memset(rx, 0, sizeof(rx));
    memset(tx, 0xff, sizeof(tx));
    tx[0] = reg | SPI_READ_BIT;

    /* Manual CS control */
    icm42688p_cs_select(devid, true);
    usleep(1);

    struct spi_trans_s trans =
    {
        .deselect = false,
        .delay    = 0,
        .nwords   = 2,
        .txbuffer = tx,
        .rxbuffer = rx
    };

    struct spi_sequence_s seq =
    {
        .ntrans = 1,
        .trans  = &trans
    };

    ret = ioctl(fd, SPIIOC_TRANSFER, (unsigned long)&seq);
    
    usleep(1);
    icm42688p_cs_select(devid, false);

    if (ret < 0)
    {
        snerr("ERROR: SPI read failed: %s\n", strerror(errno));
        return -errno;
    }

    *value = rx[1];
    return OK;
}

int icm42688p_read_registers(int fd, int devid, uint8_t reg, 
                             uint8_t *buffer, size_t len)
{
    int ret;
    size_t i;
    uint8_t *tx;
    uint8_t *rx;

    if (!buffer || len == 0)
    {
        return -EINVAL;
    }

    /* Allocate buffers */
    tx = malloc(len + 1);
    rx = malloc(len + 1);
    if (!tx || !rx)
    {
        free(tx);
        free(rx);
        return -ENOMEM;
    }

    memset(rx, 0, len + 1);
    tx[0] = reg | SPI_READ_BIT;
    for (i = 1; i <= len; i++)
    {
        tx[i] = 0xFF;
    }

    /* Manual CS control */
    icm42688p_cs_select(devid, true);
    usleep(1);

    struct spi_trans_s trans =
    {
        .deselect = false,
        .delay    = 0,
        .nwords   = (unsigned int)(len + 1),
        .txbuffer = tx,
        .rxbuffer = rx
    };

    struct spi_sequence_s seq =
    {
        .ntrans = 1,
        .trans  = &trans
    };

    ret = ioctl(fd, SPIIOC_TRANSFER, (unsigned long)&seq);
    
    usleep(1);
    icm42688p_cs_select(devid, false);

    if (ret < 0)
    {
        snerr("ERROR: SPI burst read failed: %s\n", strerror(errno));
        free(tx);
        free(rx);
        return -errno;
    }

    /* Copy output bytes (skip rx[0]) */
    for (i = 0; i < len; i++)
    {
        buffer[i] = rx[i + 1];
    }

    free(tx);
    free(rx);
    return OK;
}

int icm42688p_select_bank(int fd, int devid, uint8_t bank)
{
    return icm42688p_write_register(fd, devid, ICM42688P_REG_BANK_SEL, 
                                     bank & 0x07);
}