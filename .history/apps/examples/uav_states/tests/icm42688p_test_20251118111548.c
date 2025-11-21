/****************************************************************************
 * apps/examples/icm42688p_test/icm42688p_test.c
 *
 * Test cực kỳ đơn giản cho ICM-42688-P
 * Chỉ làm 3 việc: init → đọc WHOAMI → in dữ liệu liên tục
 *
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <unistd.h>

#include "../drivers/sensors/icm42688p/icm42688p_driver.h"

/* Lựa chọn ICM để test */
#define ICM42688P_SPI_BUS    1       /* Bus SPI sử dụng */
#define ICM42688P_SPI_CS     0       /* Chip Select SPI sử dụng */

int icm42688p_test_main(int argc, char *argv[])
{
    icm42688p_dev_t dev = {0};
    spi_dev_t spi = {0};
    icm42688p_data_t data;
    uint8_t whoami = 0;
    int ret;

    /* Đảm bảo xuống dòng sạch khi chạy từ NSH */
    printf("\n");
    fflush(stdout);

    printf("=== ICM-42688-P Test (Bus SPI%d CS%d) ===\n\n", ICM42688P_SPI_BUS, ICM42688P_SPI_CS);

    /* 1. Khởi tạo */

    ret = spi_init(&spi, ICM42688P_SPI_BUS, ICM42688P_SPI_CS);
    if (ret != SPI_OK)
    {
        printf("SPI INIT FAILED! Mã lỗi = %d\n", ret);
        return -1;
    }

    dev.spi = spi;

    ret = icm42688p_init(&dev, ICM42688P_SPI_BUS, ICM42688P_SPI_CS);
    if (ret != ICM42688P_OK)
    {
        printf("INIT FAILED! Mã lỗi = %d\n", ret);
        printf("→ Kiểm tra dây SPI, nguồn 3.3V, và config SPI trong menuconfig!\n");
        return -1;
    }

    /* 2. Đọc WHOAMI để chắc chắn nói chuyện được */
    ret = spi_read_reg(&dev, 0x75, &whoami);   // Register WHO_AM_I
    if (ret != ICM42688P_OK || whoami != 0x47)
    {
        printf("WHOAMI sai hoặc lỗi đọc! Đọc được = 0x%02X (phải là 0x47)\n", whoami);
        icm42688p_deinit(&dev);
        return -1;
    }

    printf("TÌM THẤY ICM-42688-P! WHOAMI = 0x%02X PASSED\n\n", whoami);

    /* 3. Bắt đầu in dữ liệu 1000 Hz (Ctrl+C để dừng) */
    printf("In dữ liệu Accel/Gyro/Temp 1000Hz...\n");
    printf("  Accel (g)          Gyro (dps)         Temp (°C)\n");
    printf("  X       Y       Z     X       Y       Z\n");
    printf("----------------------------------------------------\n");
    fflush(stdout);

    while (1)
    {
        ret = icm42688p_read_data(&dev, &data);
        if (ret != ICM42688P_OK)
        {
            printf("\rLỗi đọc dữ liệu!             ");
            usleep(10000);
            continue;
        }

        printf("\r%+7.3f %+7.3f %+7.3f  %+7.2f %+7.2f %+7.2f  %6.2f",
               data.accel.x,  data.accel.y,  data.accel.z,
               data.gyro.x,   data.gyro.y,   data.gyro.z,
               data.temperature);

        fflush(stdout);
        usleep(900);        // ~1000 Hz (1000000/1100 ≈ 909µs)
    }

    /* Không bao giờ tới đây */
    icm42688p_deinit(&dev);
    return 0;
}