/****************************************************************************
 * apps/examples/icm42688p_simple_test.c
 * Siêu đơn giản - chỉ để kiểm tra sensor có hoạt động không
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <unistd.h>

#include "../drivers/sensors/icm42688p/icm42688p_driver.h"

#ifndef CONFIG_UAV_STATES_ICM42688P_SPI_BUS
#  define CONFIG_UAV_STATES_ICM42688P_SPI_BUS 1
#endif
#ifndef CONFIG_UAV_STATES_ICM42688P_DEVID
#  define CONFIG_UAV_STATES_ICM42688P_DEVID 0
#endif

int icm42688p_simple_test_main(int argc, char *argv[])
{
    icm42688p_dev_t dev = {0};
    icm42688p_data_t data;
    uint8_t whoami;
    int ret;

    printf("\n=== ICM-42688-P Siêu Đơn Giản Test ===\n\n");

    ret = icm42688p_init(&dev,
                         CONFIG_UAV_STATES_ICM42688P_SPI_BUS,
                         CONFIG_UAV_STATES_ICM42688P_DEVID);

    if (ret != ICM42688P_OK) {
        printf("KHÔNG kết nối được ICM-42688-P! (lỗi = %d)\n", ret);
        printf("Kiểm tra:\n");
        printf("  • Dây SPI (SCK, MOSI, MISO, CS)\n");
        printf("  • Nguồn 3.3V cho sensor\n");
        printf("  • Config SPI bus %d có bật trong menuconfig chưa?\n",
               CONFIG_UAV_STATES_ICM42688P_SPI_BUS);
        return -1;
    }

    icm42688p_read_reg(&dev, 0x75, &whoami);  // WHOAMI register
    printf("Tìm thấy ICM-42688-P! WHOAMI = 0x%02X (phải là 0x47)\n\n", whoami);

    printf("Bắt đầu in dữ liệu 1000Hz (Ctrl+C để dừng)\n");
    printf("   Accel (g)        Gyro (dps)       Temp\n");
    printf(" X      Y      Z    X      Y      Z     °C\n");
    printf("--------------------------------------------\n");
    fflush(stdout);

    while (1) {
        ret = icm42688p_read_data(&dev, &data);
        if (ret != ICM42688P_OK) {
            printf("Read lỗi!\n");
            usleep(10000);
            continue;
        }

        printf("\r%+6.3f %+6.3f %+6.3f  %+7.2f %+7.2f %+7.2f  %6.2f",
               data.accel.x,  data.accel.y,  data.accel.z,
               data.gyro.x,   data.gyro.y,   data.gyro.z,
               data.temperature);

        fflush(stdout);
        usleep(1000);   // ~1000Hz
    }

    return 0;
}