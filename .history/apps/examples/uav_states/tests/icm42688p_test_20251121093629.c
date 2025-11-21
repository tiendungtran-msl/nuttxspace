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
#define ICM42688P_DEV1_ID              1

int icm42688p_test_main(int argc, char *argv[])
{
    icm42688p_dev_t dev0 = {0};
    uint8_t whoami = 0;
    int ret;

    /* Đảm bảo xuống dòng sạch khi chạy từ NSH */
    printf("\n");
    fflush(stdout);

    printf("=== ICM-42688-P Test ( CS%d) ===\n\n", ICM42688P_DEV0_ID);

    /* 1. Khởi tạo */
    ret = icm42688p_init(&dev0, ICM42688P_DEV0_ID);
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
        ret = icm42688p_read_data(&dev0);
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
               dev0.accel.x,  dev0.accel.y,  dev0.accel.z,
               dev0.gyro.x,   dev0.gyro.y,   dev0.gyro.z,
               dev0.temperature);

        fflush(stdout);
    }

    return 0;
}