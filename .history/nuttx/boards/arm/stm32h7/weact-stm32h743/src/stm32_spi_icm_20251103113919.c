/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/spi/spi.h>

#ifdef CONFIG_SPI_DRIVER
#  include <nuttx/spi/spi_transfer.h>
#endif

#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "stm32_gpio.h"
#include "stm32_spi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* GPIO Configuration for ICM42688P CS pins */

#define GPIO_SPI1_CS_ICM0    (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                              GPIO_OUTPUT | GPIO_PORTA | GPIO_PIN4)

#define GPIO_SPI1_CS_ICM1    (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                              GPIO_OUTPUT | GPIO_PORTD | GPIO_PIN11)

#define GPIO_SPI1_CS_ICM2    (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                              GPIO_OUTPUT | GPIO_PORTD | GPIO_PIN12)

#define GPIO_SPI1_CS_ICM3    (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                              GPIO_OUTPUT | GPIO_PORTD | GPIO_PIN13)

/* SPI Configuration for ICM42688P */

#define ICM42688P_SPI_FREQUENCY     10000000  /* 10 MHz */
#define ICM42688P_SPI_MODE          SPIDEV_MODE3

/* Number of ICM42688P sensors */

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

/* Wrapper devices for each sensor */

static struct icm42688p_spi_wrapper_s g_icm_wrappers[NUM_ICM42688P];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: icm_spi_lock
 ****************************************************************************/

static int icm_spi_lock(struct spi_dev_s *dev, bool lock)
{
  struct icm42688p_spi_wrapper_s *priv = 
    (struct icm42688p_spi_wrapper_s *)dev;
  
  return SPI_LOCK(priv->parent, lock);
}

/****************************************************************************
 * Name: icm_spi_select
 ****************************************************************************/

static void icm_spi_select(struct spi_dev_s *dev, uint32_t devid,
                           bool selected)
{
  struct icm42688p_spi_wrapper_s *priv = 
    (struct icm42688p_spi_wrapper_s *)dev;
  
  board_spi1_icm_select(priv->sensor_id, selected);
}

/****************************************************************************
 * Name: icm_spi_setfrequency
 ****************************************************************************/

static uint32_t icm_spi_setfrequency(struct spi_dev_s *dev,
                                     uint32_t frequency)
{
  struct icm42688p_spi_wrapper_s *priv = 
    (struct icm42688p_spi_wrapper_s *)dev;
  
  return SPI_SETFREQUENCY(priv->parent, frequency);
}

/****************************************************************************
 * Name: icm_spi_setmode
 ****************************************************************************/

static void icm_spi_setmode(struct spi_dev_s *dev, enum spi_mode_e mode)
{
  struct icm42688p_spi_wrapper_s *priv = 
    (struct icm42688p_spi_wrapper_s *)dev;
  
  SPI_SETMODE(priv->parent, mode);
}

/****************************************************************************
 * Name: icm_spi_setbits
 ****************************************************************************/

static void icm_spi_setbits(struct spi_dev_s *dev, int nbits)
{
  struct icm42688p_spi_wrapper_s *priv = 
    (struct icm42688p_spi_wrapper_s *)dev;
  
  SPI_SETBITS(priv->parent, nbits);
}

/****************************************************************************
 * Name: icm_spi_status
 ****************************************************************************/

static uint8_t icm_spi_status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}

/****************************************************************************
 * Name: icm_spi_send
 ****************************************************************************/

static uint32_t icm_spi_send(struct spi_dev_s *dev, uint32_t wd)
{
  struct icm42688p_spi_wrapper_s *priv = 
    (struct icm42688p_spi_wrapper_s *)dev;
  
  return SPI_SEND(priv->parent, wd);
}

/****************************************************************************
 * Name: icm_spi_exchange
 ****************************************************************************/

static void icm_spi_exchange(struct spi_dev_s *dev,
                             const void *txbuffer,
                             void *rxbuffer, size_t nwords)
{
  struct icm42688p_spi_wrapper_s *priv = 
    (struct icm42688p_spi_wrapper_s *)dev;
  
  SPI_EXCHANGE(priv->parent, txbuffer, rxbuffer, nwords);
}

/****************************************************************************
 * Name: icm_spi_sndblock
 ****************************************************************************/

#ifndef CONFIG_SPI_EXCHANGE
static void icm_spi_sndblock(struct spi_dev_s *dev,
                             const void *txbuffer, size_t nwords)
{
  struct icm42688p_spi_wrapper_s *priv = 
    (struct icm42688p_spi_wrapper_s *)dev;
  
  SPI_SNDBLOCK(priv->parent, txbuffer, nwords);
}

/****************************************************************************
 * Name: icm_spi_recvblock
 ****************************************************************************/

static void icm_spi_recvblock(struct spi_dev_s *dev,
                              void *rxbuffer, size_t nwords)
{
  struct icm42688p_spi_wrapper_s *priv = 
    (struct icm42688p_spi_wrapper_s *)dev;
  
  SPI_RECVBLOCK(priv->parent, rxbuffer, nwords);
}
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct spi_ops_s g_icm_spi_ops =
{
  .lock         = icm_spi_lock,
  .select       = icm_spi_select,
  .setfrequency = icm_spi_setfrequency,
  .setmode      = icm_spi_setmode,
  .setbits      = icm_spi_setbits,
  .status       = icm_spi_status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata      = NULL,
#endif
  .send         = icm_spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange     = icm_spi_exchange,
#else
  .sndblock     = icm_spi_sndblock,
  .recvblock    = icm_spi_recvblock,
#endif
#ifdef CONFIG_SPI_TRIGGER
  .trigger      = NULL,
#endif
#ifdef CONFIG_SPI_CALLBACK
  .registercallback = NULL,
#endif
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_spidev_icm_initialize
 ****************************************************************************/

void stm32_spidev_icm_initialize(void)
{
  int i;

  _info("Configuring CS pins for %d ICM42688P sensors\n", NUM_ICM42688P);

  for (i = 0; i < NUM_ICM42688P; i++)
    {
      stm32_configgpio(g_icm_cs_pins[i]);
      stm32_gpiowrite(g_icm_cs_pins[i], 1);
      _info("  ICM%d CS pin configured\n", i);
    }

  _info("ICM42688P CS pins initialization completed\n");
}

/****************************************************************************
 * Name: board_spi1_icm_initialize
 ****************************************************************************/

int board_spi1_icm_initialize(void)
{
  int i;

  _info("Starting SPI1 initialization for ICM42688P\n");

  /* Get SPI1 bus instance */

  g_spi1_icm = stm32_spibus_initialize(1);
  if (g_spi1_icm == NULL)
    {
      _err("ERROR: Failed to initialize SPI1 bus\n");
      return -ENODEV;
    }

  _info("SPI1 bus handle obtained\n");

  /* Configure SPI1 parameters */

  SPI_LOCK(g_spi1_icm, true);
  SPI_SETMODE(g_spi1_icm, ICM42688P_SPI_MODE);
  SPI_SETBITS(g_spi1_icm, 8);
  SPI_SETFREQUENCY(g_spi1_icm, ICM42688P_SPI_FREQUENCY);
  SPI_LOCK(g_spi1_icm, false);

  _info("SPI1 configuration completed\n");

  /* Initialize wrapper devices */

  for (i = 0; i < NUM_ICM42688P; i++)
    {
      g_icm_wrappers[i].spidev.ops = &g_icm_spi_ops;
      g_icm_wrappers[i].parent = g_spi1_icm;
      g_icm_wrappers[i].sensor_id = i;
    }

  return OK;
}

/****************************************************************************
 * Name: board_spi1_icm_select
 ****************************************************************************/

void board_spi1_icm_select(uint8_t id, bool selected)
{
  if (id < NUM_ICM42688P)
    {
      stm32_gpiowrite(g_icm_cs_pins[id], !selected);
    }
}

/****************************************************************************
 * Name: board_spi1_icm_get_handle
 ****************************************************************************/

struct spi_dev_s *board_spi1_icm_get_handle(void)
{
  return g_spi1_icm;
}

/****************************************************************************
 * Name: board_spi1_icm_get_device
 ****************************************************************************/

struct spi_dev_s *board_spi1_icm_get_device(uint8_t id)
{
  if (id >= NUM_ICM42688P)
    {
      return NULL;
    }

  return &g_icm_wrappers[id].spidev;
}