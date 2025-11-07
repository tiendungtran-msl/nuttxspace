/****************************************************************************
 * examples/stm32_gpio_control/src/gpio_hal.c
 *
 * Hardware Abstraction Layer Implementation
 * Triển khai lớp HAL - Giao tiếp trực tiếp với NuttX GPIO subsystem
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

#include <nuttx/ioexpander/gpio.h>
#include "../include/gpio_hal.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gpio_hal_validate_handle
 *
 * Description:
 *   Kiểm tra tính hợp lệ của GPIO handle
 *
 * Input Parameters:
 *   handle - GPIO HAL handle to validate
 *
 * Returned Value:
 *   true if valid, false otherwise
 ****************************************************************************/

static bool gpio_hal_validate_handle(gpio_hal_handle_t *handle)
{
  if (handle == NULL)
    {
      gpio_info("Handle is NULL\n");
      return false;
    }

  if (!handle->initialized)
    {
      gpio_info("Handle not initialized\n");
      return false;
    }

  if (handle->fd < 0)
    {
      gpio_info("Invalid file descriptor: %d\n", handle->fd);
      return false;
    }

  return true;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gpio_hal_init
 ****************************************************************************/

/****************************************************************************
 * Name: gpio_hal_init
 ****************************************************************************/

int gpio_hal_init(gpio_hal_handle_t *handle, int port, int pin, 
                  const char *device_path)
{
  int ret;
  const char* device_paths[] = {
    APP_GPIO_DEVICE_PATH,
    APP_GPIO_DEVICE_PATH_ALT1, 
    APP_GPIO_DEVICE_PATH_ALT2,
    NULL
  };
  int i;

  if (handle == NULL || device_path == NULL)
    {
      gpio_info("Invalid parameters\n");
      return GPIO_HAL_ERROR;
    }

  /* Initialize handle structure */
  memset(handle, 0, sizeof(gpio_hal_handle_t));
  handle->port = port;
  handle->pin = pin;
  handle->fd = -1;

  /* 
   * Try different device paths as NuttX GPIO device naming can vary
   * Common patterns: /dev/gpin<n>, /dev/gpout<n>, /dev/gpio<n>
   */
  for (i = 0; device_paths[i] != NULL; i++)
    {
      strncpy(handle->device_path, device_paths[i], sizeof(handle->device_path) - 1);
      handle->device_path[sizeof(handle->device_path) - 1] = '\0';
      
      gpio_info("Trying to open GPIO device: %s\n", device_paths[i]);
      
      /* Try to open GPIO device file with error checking */
      handle->fd = open(device_paths[i], O_RDWR);
      if (handle->fd >= 0)
        {
          gpio_info("Successfully opened GPIO device: %s (fd=%d)\n", 
                    device_paths[i], handle->fd);
          break;
        }
      else
        {
          gpio_info("Failed to open %s: errno=%d (%s)\n", 
                    device_paths[i], errno, strerror(errno));
        }
    }

  /* If all device paths failed */
  if (handle->fd < 0)
    {
      gpio_info("ERROR: All GPIO device paths failed. Available devices:\n");
      gpio_info("Please check 'ls /dev/gp*' or 'ls /dev/gpio*'\n");
      gpio_info("Make sure GPIO driver is enabled in NuttX config\n");
      return GPIO_HAL_ERROR;
    }

  /* 
   * Configure GPIO as output with safer approach
   * Some NuttX GPIO drivers don't support GPIOC_SETPINTYPE
   * Try different configuration methods
   */
  ret = ioctl(handle->fd, GPIOC_SETPINTYPE, GPIO_OUTPUT_PIN);
  if (ret < 0)
    {
      gpio_info("GPIOC_SETPINTYPE failed (errno=%d), trying alternative method\n", errno);
      
      /* Try writing a test value to see if it's already configured */
      char test_val = '0';
      ssize_t bytes = write(handle->fd, &test_val, 1);
      if (bytes != 1)
        {
          gpio_info("GPIO write test failed, device may not support output mode\n");
          close(handle->fd);
          handle->fd = -1;
          return GPIO_HAL_ERROR;
        }
      else
        {
          gpio_info("GPIO device accepts write operations (pre-configured as output)\n");
        }
    }
  else
    {
      gpio_info("GPIO configured as output pin successfully\n");
    }

  handle->initialized = true;
  
  gpio_info("GPIO HAL initialized - Port: %c, Pin: %d, Device: %s, FD: %d\n",
            'A' + port, pin, handle->device_path, handle->fd);
  
  return GPIO_HAL_OK;
}

/****************************************************************************
 * Name: gpio_hal_write
 ****************************************************************************/

int gpio_hal_write(gpio_hal_handle_t *handle, bool value)
{
  if (!gpio_hal_validate_handle(handle))
    {
      return GPIO_HAL_ERROR;
    }

  /* 
   * Ghi giá trị vào GPIO pin với error checking cẩn thận
   * NuttX GPIO driver hỗ trợ write() operation
   * value: true = HIGH (1), false = LOW (0)
   */
  char gpio_value = value ? '1' : '0';
  
  /* Ensure we're at the beginning of the device */
  if (lseek(handle->fd, 0, SEEK_SET) < 0)
    {
      gpio_info("lseek failed: errno=%d (%s)\n", errno, strerror(errno));
      /* Continue anyway, some GPIO devices don't support seek */
    }
  
  ssize_t bytes_written = write(handle->fd, &gpio_value, 1);
  
  if (bytes_written != 1)
    {
      gpio_info("Failed to write GPIO value (wrote %d bytes): errno=%d (%s)\n", 
                (int)bytes_written, errno, strerror(errno));
      return GPIO_HAL_ERROR;
    }

  /* Force flush to ensure write reaches hardware */
  fsync(handle->fd);

  gpio_info("GPIO P%c%d set to %s (fd=%d)\n", 
            'A' + handle->port, handle->pin, value ? "HIGH" : "LOW", handle->fd);
  
  return GPIO_HAL_OK;
}

/****************************************************************************
 * Name: gpio_hal_read
 ****************************************************************************/

int gpio_hal_read(gpio_hal_handle_t *handle, bool *value)
{
  if (!gpio_hal_validate_handle(handle) || value == NULL)
    {
      return GPIO_HAL_ERROR;
    }

  /* 
   * Đọc giá trị từ GPIO pin với error handling
   */
  char gpio_value = '0';
  
  /* Ensure we're at the beginning of the device */
  if (lseek(handle->fd, 0, SEEK_SET) < 0)
    {
      gpio_info("lseek failed during read: errno=%d (%s)\n", errno, strerror(errno));
      /* Continue anyway */
    }
  
  ssize_t bytes_read = read(handle->fd, &gpio_value, 1);
  
  if (bytes_read != 1)
    {
      gpio_info("Failed to read GPIO value (read %d bytes): errno=%d (%s)\n", 
                (int)bytes_read, errno, strerror(errno));
      return GPIO_HAL_ERROR;
    }

  /* Convert character to boolean, handle various formats */
  if (gpio_value == '1' || gpio_value == 1)
    {
      *value = true;
    }
  else if (gpio_value == '0' || gpio_value == 0)
    {
      *value = false;
    }
  else
    {
      gpio_info("Unexpected GPIO value read: 0x%02x ('%c')\n", gpio_value, gpio_value);
      /* Assume any non-zero value is HIGH */
      *value = (gpio_value != '0' && gpio_value != 0);
    }
  
  gpio_info("GPIO P%c%d read as %s (raw: 0x%02x)\n", 
            'A' + handle->port, handle->pin, *value ? "HIGH" : "LOW", gpio_value);
  
  return GPIO_HAL_OK;
}

/****************************************************************************
 * Name: gpio_hal_toggle
 ****************************************************************************/

int gpio_hal_toggle(gpio_hal_handle_t *handle)
{
  bool current_value;
  int ret;

  if (!gpio_hal_validate_handle(handle))
    {
      return GPIO_HAL_ERROR;
    }

  /* Đọc trạng thái hiện tại */
  ret = gpio_hal_read(handle, &current_value);
  if (ret != GPIO_HAL_OK)
    {
      gpio_info("Failed to read current GPIO state for toggle\n");
      return GPIO_HAL_ERROR;
    }

  /* Đảo trạng thái */
  ret = gpio_hal_write(handle, !current_value);
  if (ret != GPIO_HAL_OK)
    {
      gpio_info("Failed to write toggled GPIO state\n");
      return GPIO_HAL_ERROR;
    }

  gpio_info("GPIO P%c%d toggled from %s to %s\n", 
            'A' + handle->port, handle->pin,
            current_value ? "HIGH" : "LOW",
            !current_value ? "HIGH" : "LOW");
  
  return GPIO_HAL_OK;
}

/****************************************************************************
 * Name: gpio_hal_deinit
 ****************************************************************************/

int gpio_hal_deinit(gpio_hal_handle_t *handle)
{
  if (handle == NULL)
    {
      return GPIO_HAL_ERROR;
    }

  if (handle->initialized && handle->fd >= 0)
    {
      /* 
       * Đóng GPIO device file
       * NuttX VFS sẽ tự động giải phóng resources
       */
      close(handle->fd);
      gpio_info("GPIO device %s closed\n", handle->device_path);
    }

  /* Reset handle state */
  memset(handle, 0, sizeof(gpio_hal_handle_t));
  handle->fd = -1;
  
  gpio_info("GPIO HAL deinitialized\n");
  
  return GPIO_HAL_OK;
}