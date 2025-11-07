/****************************************************************************
 * examples/stm32_gpio_control/include/gpio_driver.h
 *
 * GPIO Driver Layer
 * Lớp driver - Cung cấp interface cho application logic
 ****************************************************************************/

#ifndef __EXAMPLES_STM32_GPIO_CONTROL_GPIO_DRIVER_H
#define __EXAMPLES_STM32_GPIO_CONTROL_GPIO_DRIVER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdbool.h>
#include "gpio_hal.h"
#include "app_config.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GPIO_DRIVER_OK       0
#define GPIO_DRIVER_ERROR   -1

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* GPIO Driver Context - Quản lý trạng thái driver */
typedef struct
{
  gpio_hal_handle_t hal_handle;  /* HAL handle */
  app_state_t state;            /* Driver state */
  int error_count;              /* Error counter for debugging */
  uint32_t operation_count;     /* Total operations performed */
} gpio_driver_context_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: gpio_driver_init
 *
 * Description:
 *   Khởi tạo GPIO driver
 *   - Khởi tạo HAL layer
 *   - Thiết lập driver context
 *   - Liên kết với NuttX GPIO subsystem
 *
 * Input Parameters:
 *   ctx - Pointer to driver context
 *
 * Returned Value:
 *   GPIO_DRIVER_OK on success, GPIO_DRIVER_ERROR on failure
 ****************************************************************************/

int gpio_driver_init(gpio_driver_context_t *ctx);

/****************************************************************************
 * Name: gpio_driver_execute_command
 *
 * Description:
 *   Thực thi lệnh GPIO
 *   - Xử lý các lệnh từ application layer
 *   - Gọi các hàm HAL tương ứng
 *   - Quản lý trạng thái và error handling
 *
 * Input Parameters:
 *   ctx - Driver context
 *   cmd - GPIO command to execute
 *   value - Value parameter (for SET commands)
 *   result - Pointer to store result (for READ command)
 *
 * Returned Value:
 *   GPIO_DRIVER_OK on success, GPIO_DRIVER_ERROR on failure
 ****************************************************************************/

int gpio_driver_execute_command(gpio_driver_context_t *ctx, 
                               gpio_cmd_t cmd, 
                               bool value, 
                               bool *result);

/****************************************************************************
 * Name: gpio_driver_get_status
 *
 * Description:
 *   Lấy thông tin trạng thái driver
 *   - Trạng thái hiện tại
 *   - Số lượng lỗi
 *   - Số lượng operations đã thực hiện
 *
 * Input Parameters:
 *   ctx - Driver context
 *   state - Pointer to store current state
 *   error_count - Pointer to store error count
 *   op_count - Pointer to store operation count
 *
 * Returned Value:
 *   GPIO_DRIVER_OK on success, GPIO_DRIVER_ERROR on failure
 ****************************************************************************/

int gpio_driver_get_status(gpio_driver_context_t *ctx,
                          app_state_t *state,
                          int *error_count,
                          uint32_t *op_count);

/****************************************************************************
 * Name: gpio_driver_cleanup
 *
 * Description:
 *   Dọn dẹp tài nguyên driver
 *   - Giải phóng HAL resources
 *   - Reset driver context
 *
 * Input Parameters:
 *   ctx - Driver context
 *
 * Returned Value:
 *   GPIO_DRIVER_OK on success, GPIO_DRIVER_ERROR on failure
 ****************************************************************************/

int gpio_driver_cleanup(gpio_driver_context_t *ctx);

#endif /* __EXAMPLES_STM32_GPIO_CONTROL_GPIO_DRIVER_H */