/****************************************************************************
 * examples/stm32_gpio_control/src/gpio_driver.c
 *
 * GPIO Driver Layer Implementation  
 * Triển khai lớp driver - Cầu nối giữa application và HAL
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

#include "../include/gpio_driver.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gpio_driver_validate_context
 *
 * Description:
 *   Kiểm tra tính hợp lệ của driver context
 *
 * Input Parameters:
 *   ctx - Driver context to validate
 *
 * Returned Value:
 *   true if valid, false otherwise
 ****************************************************************************/

static bool gpio_driver_validate_context(gpio_driver_context_t *ctx)
{
  if (ctx == NULL)
    {
      gpio_info("Driver context is NULL\n");
      return false;
    }

  if (ctx->state == APP_STATE_ERROR)
    {
      gpio_info("Driver is in error state\n");
      return false;
    }

  return true;
}

/****************************************************************************
 * Name: gpio_driver_update_stats
 *
 * Description:
 *   Cập nhật thống kê driver operations
 *
 * Input Parameters:
 *   ctx - Driver context
 *   success - Operation success status
 ****************************************************************************/

static void gpio_driver_update_stats(gpio_driver_context_t *ctx, bool success)
{
  if (ctx == NULL) return;

  ctx->operation_count++;
  
  if (!success)
    {
      ctx->error_count++;
      
      /* Chuyển sang error state nếu quá nhiều lỗi */
      if (ctx->error_count > 5)
        {
          ctx->state = APP_STATE_ERROR;
          gpio_info("Driver entered error state after %d errors\n", 
                    ctx->error_count);
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gpio_driver_init
 ****************************************************************************/

int gpio_driver_init(gpio_driver_context_t *ctx)
{
  int ret;

  if (ctx == NULL)
    {
      gpio_info("Driver context is NULL\n");
      return GPIO_DRIVER_ERROR;
    }

  /* Initialize driver context */
  memset(ctx, 0, sizeof(gpio_driver_context_t));
  ctx->state = APP_STATE_INIT;

  /* 
   * Khởi tạo HAL layer
   * Liên kết với NuttX GPIO subsystem thông qua HAL
   */
  ret = gpio_hal_init(&ctx->hal_handle, 
                      APP_GPIO_PORT, 
                      APP_GPIO_PIN,
                      APP_GPIO_DEVICE_PATH);
  
  if (ret != GPIO_HAL_OK)
    {
      gpio_info("Failed to initialize GPIO HAL\n");
      ctx->state = APP_STATE_ERROR;
      gpio_driver_update_stats(ctx, false);
      return GPIO_DRIVER_ERROR;
    }

  ctx->state = APP_STATE_RUNNING;
  gpio_driver_update_stats(ctx, true);
  
  gpio_info("GPIO Driver initialized successfully\n");
  gpio_info("Controlling STM32F411 BlackPill PC13 (Built-in LED)\n");
  
  return GPIO_DRIVER_OK;
}

/****************************************************************************
 * Name: gpio_driver_execute_command
 ****************************************************************************/

int gpio_driver_execute_command(gpio_driver_context_t *ctx, 
                               gpio_cmd_t cmd, 
                               bool value, 
                               bool *result)
{
  int ret = GPIO_DRIVER_ERROR;

  if (!gpio_driver_validate_context(ctx))
    {
      return GPIO_DRIVER_ERROR;
    }

  /* 
   * Thực thi lệnh dựa trên command type
   * Gọi các hàm HAL tương ứng
   */
  switch (cmd)
    {
      case GPIO_CMD_INIT:
        gpio_info("GPIO already initialized\n");
        ret = GPIO_DRIVER_OK;
        break;

      case GPIO_CMD_SET_HIGH:
        gpio_info("Executing SET_HIGH command\n");
        ret = gpio_hal_write(&ctx->hal_handle, true);
        break;

      case GPIO_CMD_SET_LOW:
        gpio_info("Executing SET_LOW command\n");
        ret = gpio_hal_write(&ctx->hal_handle, false);
        break;

      case GPIO_CMD_TOGGLE:
        gpio_info("Executing TOGGLE command\n");
        ret = gpio_hal_toggle(&ctx->hal_handle);
        break;

      case GPIO_CMD_READ:
        if (result != NULL)
          {
            gpio_info("Executing READ command\n");
            ret = gpio_hal_read(&ctx->hal_handle, result);
          }
        else
          {
            gpio_info("READ command requires result pointer\n");
            ret = GPIO_DRIVER_ERROR;
          }
        break;

      case GPIO_CMD_CLEANUP:
        gpio_info("Executing CLEANUP command\n");
        ret = gpio_hal_deinit(&ctx->hal_handle);
        if (ret == GPIO_HAL_OK)
          {
            ctx->state = APP_STATE_STOPPED;
          }
        break;

      default:
        gpio_info("Unknown GPIO command: %d\n", cmd);
        ret = GPIO_DRIVER_ERROR;
        break;
    }

  /* Cập nhật thống kê operations */
  gpio_driver_update_stats(ctx, (ret == GPIO_DRIVER_OK));

  return (ret == GPIO_HAL_OK) ? GPIO_DRIVER_OK : GPIO_DRIVER_ERROR;
}

/****************************************************************************
 * Name: gpio_driver_get_status
 ****************************************************************************/

int gpio_driver_get_status(gpio_driver_context_t *ctx,
                          app_state_t *state,
                          int *error_count,
                          uint32_t *op_count)
{
  if (ctx == NULL)
    {
      return GPIO_DRIVER_ERROR;
    }

  if (state != NULL)
    {
      *state = ctx->state;
    }

  if (error_count != NULL)
    {
      *error_count = ctx->error_count;
    }

  if (op_count != NULL)
    {
      *op_count = ctx->operation_count;
    }

  return GPIO_DRIVER_OK;
}

/****************************************************************************
 * Name: gpio_driver_cleanup
 ****************************************************************************/

int gpio_driver_cleanup(gpio_driver_context_t *ctx)
{
  int ret;

  if (ctx == NULL)
    {
      return GPIO_DRIVER_ERROR;
    }

  /* Dọn dẹp HAL resources */
  ret = gpio_hal_deinit(&ctx->hal_handle);
  
  /* Reset driver context */
  ctx->state = APP_STATE_STOPPED;
  
  gpio_info("GPIO Driver cleanup completed. Stats: ops=%lu, errors=%d\n",
            ctx->operation_count, ctx->error_count);

  return (ret == GPIO_HAL_OK) ? GPIO_DRIVER_OK : GPIO_DRIVER_ERROR;
}