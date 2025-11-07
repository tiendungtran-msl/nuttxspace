/**
 * @file stm32f411_pc13.c
 * @brief Driver điều khiển chân PC13 trên STM32F411-minimum
 * 
 * File này nằm trong: nuttx/boards/arm/stm32/stm32f411-minimum/src/
 * 
 * NGUYÊN LÝ HOẠT ĐỘNG:
 * - PC13 là chân GPIO thuộc Port C, bit 13
 * - Trên board STM32F411-minimum, PC13 thường được nối với LED onboard
 * - Driver này cung cấp các hàm cơ bản để điều khiển trạng thái ON/OFF của chân PC13
 * - Sử dụng HAL của NuttX để tương tác với hardware
 */

#include <nuttx/config.h>
#include <sys/types.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/fs/fs.h>
#include <nuttx/ioexpander/gpio.h>

#include <arch/board/board.h>
#include "chip.h"
#include "stm32.h"
#include "stm32f411-minimum.h"

/**
 * ĐỊNH NGHĨA CẤU HÌNH CHÂN PC13
 * 
 * GPIO_PC13: Macro định nghĩa cấu hình cho chân PC13
 * - GPIO_OUTPUT: Chế độ output (xuất tín hiệu)
 * - GPIO_PUSHPULL: Kiểu push-pull (có thể kéo lên VCC hoặc xuống GND)
 * - GPIO_SPEED_50MHz: Tốc độ chuyển đổi tối đa 50MHz
 * - GPIO_OUTPUT_CLEAR: Trạng thái ban đầu là LOW (0V)
 * - GPIO_PORTC: Thuộc Port C
 * - GPIO_PIN13: Chân số 13
 */
#define GPIO_PC13  (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                    GPIO_OUTPUT_CLEAR | GPIO_PORTC | GPIO_PIN13)

/**
 * @brief Cấu trúc dữ liệu lưu trữ trạng thái của PC13
 */
struct pc13_dev_s
{
    bool initialized;    /* Đã được khởi tạo chưa */
    bool state;         /* Trạng thái hiện tại: true=HIGH, false=LOW */
};

/* Biến toàn cục lưu trạng thái device */
static struct pc13_dev_s g_pc13dev;

/**
 * @brief Khởi tạo chân PC13
 * @return 0 nếu thành công, <0 nếu có lỗi
 * 
 * NGUYÊN LÝ:
 * - Cấu hình chân PC13 thành chế độ GPIO output
 * - Thiết lập trạng thái ban đầu là LOW
 * - Đánh dấu device đã được khởi tạo
 */
int stm32_pc13_initialize(void)
{
    /* Kiểm tra xem đã khởi tạo chưa */
    if (g_pc13dev.initialized)
    {
        return OK;
    }

    /* Cấu hình chân PC13 theo định nghĩa GPIO_PC13 */
    stm32_configgpio(GPIO_PC13);
    
    /* Đặt trạng thái ban đầu là LOW (LED tắt nếu là active LOW) */
    stm32_gpiowrite(GPIO_PC13, false);
    
    /* Khởi tạo cấu trúc dữ liệu */
    g_pc13dev.initialized = true;
    g_pc13dev.state = false;
    
    gpioinfo("PC13 initialized successfully\n");
    return OK;
}

/**
 * @brief Đặt trạng thái cho chân PC13
 * @param state true để đặt HIGH (3.3V), false để đặt LOW (0V)
 * @return 0 nếu thành công, <0 nếu có lỗi
 * 
 * NGUYÊN LỦ:
 * - Sử dụng hàm stm32_gpiowrite() để ghi giá trị logic ra chân
 * - true = HIGH = 3.3V
 * - false = LOW = 0V (GND)
 */
int stm32_pc13_set_state(bool state)
{
    /* Kiểm tra xem đã khởi tạo chưa */
    if (!g_pc13dev.initialized)
    {
        gpioerr("PC13 not initialized\n");
        return -ENODEV;
    }
    
    /* Ghi giá trị ra chân PC13 */
    stm32_gpiowrite(GPIO_PC13, state);
    
    /* Cập nhật trạng thái trong cấu trúc dữ liệu */
    g_pc13dev.state = state;
    
    gpioinfo("PC13 set to %s\n", state ? "HIGH" : "LOW");
    return OK;
}

/**
 * @brief Đọc trạng thái hiện tại của chân PC13
 * @return true nếu HIGH, false nếu LOW
 * 
 * NGUYÊN LÝ:
 * - Đọc giá trị logic hiện tại từ thanh ghi GPIO
 * - Trả về trạng thái thực tế của chân, không phải trạng thái đã lưu
 */
bool stm32_pc13_get_state(void)
{
    if (!g_pc13dev.initialized)
    {
        gpioerr("PC13 not initialized\n");
        return false;
    }
    
    /* Đọc trạng thái thực tế từ hardware */
    bool current_state = stm32_gpioread(GPIO_PC13);
    
    /* Cập nhật trạng thái đã lưu */
    g_pc13dev.state = current_state;
    
    return current_state;
}

/**
 * @brief Đảo trạng thái chân PC13 (toggle)
 * @return 0 nếu thành công, <0 nếu có lỗi
 * 
 * NGUYÊN LÝ:
 * - Đọc trạng thái hiện tại
 * - Đảo ngược trạng thái (HIGH->LOW hoặc LOW->HIGH)
 * - Ghi trạng thái mới ra chân
 */
int stm32_pc13_toggle(void)
{
    if (!g_pc13dev.initialized)
    {
        gpioerr("PC13 not initialized\n");
        return -ENODEV;
    }
    
    /* Đọc trạng thái hiện tại */
    bool current_state = stm32_pc13_get_state();
    
    /* Đảo trạng thái và ghi ra chân */
    return stm32_pc13_set_state(!current_state);
}

/**
 * @brief Hủy khởi tạo PC13 (cleanup)
 * @return 0 nếu thành công
 * 
 * NGUYÊN LÝ:
 * - Đặt chân về trạng thái LOW an toàn
 * - Đánh dấu device chưa được khởi tạo
 * - Giải phóng tài nguyên nếu cần
 */
int stm32_pc13_uninitialize(void)
{
    if (!g_pc13dev.initialized)
    {
        return OK;
    }
    
    /* Đặt về trạng thái an toàn */
    stm32_gpiowrite(GPIO_PC13, false);
    
    /* Đánh dấu chưa khởi tạo */
    g_pc13dev.initialized = false;
    g_pc13dev.state = false;
    
    gpioinfo("PC13 uninitialized\n");
    return OK;
}

/**
 * @brief Kiểm tra xem PC13 đã được khởi tạo chưa
 * @return true nếu đã khởi tạo, false nếu chưa
 */
bool stm32_pc13_is_initialized(void)
{
    return g_pc13dev.initialized;
}