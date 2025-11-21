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
#include <debug.h>
#include <errno.h>
#include <syslog.h>

#include "../drivers/sensors/icm42688p/icm42688p_driver.h"

#define ICM42688P_DEV0_ID              0
#define ICM42688P_DEV1_ID              0

int icm42688p_test_main(int argc, char *argv[])
{
    icm42688p_dev_t dev = {0};
    uint8_t whoami = 0;
    int ret;

    /* Đảm bảo xuống dòng sạch khi chạy từ NSH */
    printf("\n");
    fflush(stdout);

    printf("=== ICM-42688-P Test ( CS%d) ===\n\n", ICM42688P_SPI_BUS, ICM42688P_SPI_CS);

    /* 1. Khởi tạo */
    ret = icm42688p_init(&dev, ICM42688P_SPI_BUS, ICM42688P_SPI_CS);
    if (ret != ICM42688P_OK)
    {
        syslog(LOG_ERR, "INIT ICM42688P FAILED! Mã lỗi = %d\n", ret);
        return -1;
    }

    /* 2. Bắt đầu in dữ liệu 1000 Hz (Ctrl+C để dừng) */
    printf("In dữ liệu Accel/Gyro/Temp 1000Hz...\n");
    printf("  Accel (g)          Gyro (dps)         Temp (°C)\n");
    printf("  X       Y       Z     X       Y       Z       \n");
    printf("----------------------------------------------------\n");
    fflush(stdout);
    usleep(10000);  /* Delay nhỏ để đảm bảo dữ liệu ổn định */

    while (1)
    {
        usleep(900);        // ~1000 Hz (1000000/1100 ≈ 909µs)
        ret = icm42688p_read_data(&dev, &data);
        if (ret != ICM42688P_OK)
        {
            syslog(LOG_ERR, "\rLỗi đọc dữ liệu!             ");
            usleep(10000);
            continue;
        }
        static uint32_t count = 0;
        count++;
        
        /* Chỉ log sau mỗi 500ms (500 lần thu thập với chu kỳ 1ms) */
        if (count < 500)
        {
            continue;
        }
        
        count = 0;
        syslog(LOG_INFO, "\r%+7.3f %+7.3f %+7.3f  %+7.2f %+7.2f %+7.2f  %6.2f",
               data.accel.x,  data.accel.y,  data.accel.z,
               data.gyro.x,   data.gyro.y,   data.gyro.z,
               data.temperature);

        fflush(stdout);
    }

    /* Không bao giờ tới đây */
    icm42688p_deinit(&dev);
    return 0;
}