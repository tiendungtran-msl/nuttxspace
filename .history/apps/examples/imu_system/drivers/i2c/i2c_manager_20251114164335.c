/****************************************************************************
 * apps/examples/imu_system/drivers/i2c/i2c_manager.c
 *
 * I2C Manager with mutex protection
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <pthread.h>
#include <nuttx/i2c/i2c_master.h>

#include "i2c_manager.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static int g_i2c_fd = -1;
static pthread_mutex_t g_i2c_mutex = PTHREAD_MUTEX_INITIALIZER;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int i2c_manager_init(int bus)
{
  char devpath[16];
  
  snprintf(devpath, sizeof(devpath), "/dev/i2c%d", bus);
  
  g_i2c_fd = open(devpath, O_RDWR);
  if (g_i2c_fd < 0)
    {
      return -errno;
    }
  
  return OK;
}

void i2c_manager_deinit(void)
{
  if (g_i2c_fd >= 0)
    {
      close(g_i2c_fd);
      g_i2c_fd = -1;
    }
}

int i2c_manager_read_reg(uint8_t addr, uint8_t reg, uint8_t *value)
{
  struct i2c_msg_s msg[2];
  int ret;
  
  if (g_i2c_fd < 0 || !value)
    {
      return -EINVAL;
    }
  
  pthread_mutex_lock(&g_i2c_mutex);
  
  /* Write register address */
  msg[0].frequency = 400000;
  msg[0].addr      = addr;
  msg[0].flags     = 0;
  msg[0].buffer    = &reg;
  msg[0].length    = 1;
  
  /* Read data */
  msg[1].frequency = 400000;
  msg[1].addr      = addr;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = value;
  msg[1].length    = 1;
  
  ret = ioctl(g_i2c_fd, I2CIOC_TRANSFER, (unsigned long)msg);
  
  pthread_mutex_unlock(&g_i2c_mutex);
  
  return (ret == 2) ? OK : -EIO;
}

int i2c_manager_read_regs(uint8_t addr, uint8_t reg, 
                          uint8_t *buffer, size_t len)
{
  struct i2c_msg_s msg[2];
  int ret;
  
  if (g_i2c_fd < 0 || !buffer || len == 0)
    {
      return -EINVAL;
    }
  
  pthread_mutex_lock(&g_i2c_mutex);
  
  /* Write register address */
  msg[0].frequency = 400000;
  msg[0].addr      = addr;
  msg[0].flags     = 0;
  msg[0].buffer    = &reg;
  msg[0].length    = 1;
  
  /* Read data */
  msg[1].frequency = 400000;
  msg[1].addr      = addr;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = buffer;
  msg[1].length    = len;
  
  ret = ioctl(g_i2c_fd, I2CIOC_TRANSFER, (unsigned long)msg);
  
  pthread_mutex_unlock(&g_i2c_mutex);
  
  return (ret == 2) ? OK : -EIO;
}

int i2c_manager_write_reg(uint8_t addr, uint8_t reg, uint8_t value)
{
  struct i2c_msg_s msg;
  uint8_t buffer[2];
  int ret;
  
  if (g_i2c_fd < 0)
    {
      return -EINVAL;
    }
  
  buffer[0] = reg;
  buffer[1] = value;
  
  pthread_mutex_lock(&g_i2c_mutex);
  
  msg.frequency = 400000;
  msg.addr      = addr;
  msg.flags     = 0;
  msg.buffer    = buffer;
  msg.length    = 2;
  
  ret = ioctl(g_i2c_fd, I2CIOC_TRANSFER, (unsigned long)&msg);
  
  pthread_mutex_unlock(&g_i2c_mutex);
  
  return (ret == 1) ? OK : -EIO;
}