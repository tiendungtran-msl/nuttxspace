/****************************************************************************
 * examples/stm32_gpio_control/include/gpio_hal.h
 *
 * Hardware Abstraction Layer for GPIO
 * Lớp trừu tượng hóa phần cứng - Cung cấp interface chuẩn cho GPIO
 ****************************************************************************/

#ifndef __EXAMPLES_STM32_GPIO_CONTROL_GPIO_HAL_H
#define __EXAMPLES_STM32_GPIO_CONTROL_GPIO_HAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdbool.h>
#include "app_config.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GPIO_HAL_OK       0
#define GPIO_HAL_ERROR   -1

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* GPIO HAL Handle - Đại diện cho một GPIO pin trong HAL */
typedef struct
{
  int fd;                    /* File descriptor từ NuttX GPIO driver */
  int port;                  /* GPIO Port number (A=0, B=1, C=2, ...) */
  int pin;                   /* GPIO Pin number (0-15) */
  bool initialized;          /* Trạng thái khởi tạo */
  char device_path[32];      /* Device path trong hệ thống file */
} gpio_hal_handle_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: gpio_hal_init
 *
 * Description:
 *   Khởi tạo GPIO HAL handle
 *   - Mở GPIO device file
 *   - Cấu hình GPIO mode
 *   - Liên kết với NuttX GPIO subsystem
 *
 * Input Parameters:
 *   handle - Pointer to GPIO HAL handle
 *   port   - GPIO port number
 *   pin    - GPIO pin number
 *   device_path - Device path string
 *
 * Returned Value:
 *   GPIO_HAL_OK on success, GPIO_HAL_ERROR on failure
 ****************************************************************************/

int gpio_hal_init(gpio_hal_handle_t *handle, int port, int pin, 
                  const char *device_path);

/****************************************************************************
 * Name: gpio_hal_write
 *
 * Description:
 *   Ghi giá trị logic vào GPIO pin
 *   - Gọi ioctl() để điều khiển GPIO driver
 *   - Thực hiện write operation qua NuttX GPIO framework
 *
 * Input Parameters:
 *   handle - GPIO HAL handle
 *   value  - Logic value (true = HIGH, false = LOW)
 *
 * Returned Value:
 *   GPIO_HAL_OK on success, GPIO_HAL_ERROR on failure
 ****************************************************************************/

int gpio_hal_write(gpio_hal_handle_t *handle, bool value);

/****************************************************************************
 * Name: gpio_hal_read
 *
 * Description:
 *   Đọc giá trị logic từ GPIO pin
 *   - Sử dụng read() system call qua NuttX VFS
 *   - Trả về trạng thái hiện tại của pin
 *
 * Input Parameters:
 *   handle - GPIO HAL handle
 *   value  - Pointer to store read value
 *
 * Returned Value:
 *   GPIO_HAL_OK on success, GPIO_HAL_ERROR on failure
 ****************************************************************************/

int gpio_hal_read(gpio_hal_handle_t *handle, bool *value);

/****************************************************************************
 * Name: gpio_hal_toggle
 *
 * Description:
 *   Đảo trạng thái GPIO pin (HIGH->LOW, LOW->HIGH)
 *   - Đọc trạng thái hiện tại
 *   - Ghi giá trị ngược lại
 *
 * Input Parameters:
 *   handle - GPIO HAL handle
 *
 * Returned Value:
 *   GPIO_HAL_OK on success, GPIO_HAL_ERROR on failure
 ****************************************************************************/

int gpio_hal_toggle(gpio_hal_handle_t *handle);

/****************************************************************************
 * Name: gpio_hal_deinit
 *
 * Description:
 *   Giải phóng tài nguyên GPIO HAL
 *   - Đóng file descriptor
 *   - Reset handle state
 *
 * Input Parameters:
 *   handle - GPIO HAL handle
 *
 * Returned Value:
 *   GPIO_HAL_OK on success, GPIO_HAL_ERROR on failure
 ****************************************************************************/

int gpio_hal_deinit(gpio_hal_handle_t *handle);

#endif /* __EXAMPLES_STM32_GPIO_CONTROL_GPIO_HAL_H */