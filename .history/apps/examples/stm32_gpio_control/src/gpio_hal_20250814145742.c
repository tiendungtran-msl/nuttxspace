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

int gpio_hal_init(gpio_hal_handle_t *handle, int port, int pin, 
                  const char *device_path)
{
  if (handle == NULL || device_path == NULL)
    {
      gpio_info("Invalid parameters\n");
      return GPIO_HAL_ERROR;
    }

  /* Initialize handle structure */
  memset(handle, 0, sizeof(gpio_hal_handle_t));
  handle->port = port;
  handle->pin = pin;
  strncpy(handle->device_path, device_path, sizeof(handle->device_path) - 1);
  handle->device_path[sizeof(handle->device_path) - 1] = '\0';

  /* 
   * Mở GPIO device file
   * NuttX GPIO framework cung cấp character device interface
   * qua Virtual File System (VFS)
   */
  handle->fd = open(device_path, O_RDWR);
  if (handle->fd < 0)
    {
      gpio_info("Failed to open GPIO device %s: errno=%d\n", 
                device_path, errno);
      return GPIO_HAL_ERROR;
    }

  /* 
   * Cấu hình GPIO như output
   * Sử dụng ioctl() để giao tiếp với NuttX GPIO driver
   * GPIOC_WRITE được định nghĩa trong nuttx/ioexpander/gpio.h
   */
  int ret = ioctl(handle->fd, GPIOC_SETPINTYPE, GPIO_OUTPUT_PIN);
  if (ret < 0)
    {
      gpio_info("Failed to configure GPIO as output: errno=%d\n", errno);
      close(handle->fd);
      handle->fd = -1;
      return GPIO_HAL_ERROR;
    }

  handle->initialized = true;
  
  gpio_info("GPIO HAL initialized - Port: %c, Pin: %d, Device: %s\n",
            'A' + port, pin, device_path);
  
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
   * Ghi giá trị vào GPIO pin
   * NuttX GPIO driver hỗ trợ write() operation
   * value: true = HIGH (1), false = LOW (0)
   */
  char gpio_value = value ? '1' : '0';
  ssize_t bytes_written = write(handle->fd, &gpio_value, 1);
  
  if (bytes_written != 1)
    {
      gpio_info("Failed to write GPIO value: errno=%d\n", errno);
      return GPIO_HAL_ERROR;
    }

  gpio_info("GPIO P%c%d set to %s\n", 
            'A' + handle->port, handle->pin, value ? "HIGH" : "LOW");
  
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
   * Đọc giá trị từ GPIO pin
   * Sử dụng read() system call qua NuttX VFS
   */
  char gpio_value;
  ssize_t bytes_read = read(handle->fd, &gpio_value, 1);
  
  if (bytes_read != 1)
    {
      gpio_info("Failed to read GPIO value: errno=%d\n", errno);
      return GPIO_HAL_ERROR;
    }

  *value = (gpio_value == '1');
  
  gpio_info("GPIO P%c%d read as %s\n", 
            'A' + handle->port, handle->pin, *value ? "HIGH" : "LOW");
  
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