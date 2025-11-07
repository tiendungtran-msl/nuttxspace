#include <nuttx/config.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/mpu6050.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <math.h>
#include <errno.h>
#include <sys/ioctl.h>

#define MPU6050_I2C_PORT 1
#define MPU6050_I2C_ADDR 0x68
#define RAD_TO_DEG (180.0 / M_PI)

int main(int argc, FAR char *argv[])
{
    int fd;
    struct mpu6050_data_s data;
    float roll, pitch;
    int ret;

    /* Mở thiết bị MPU6050 */
    fd = open("/dev/mpu6050", O_RDWR);
    if (fd < 0)
    {
        printf("Không thể mở /dev/mpu6050: %d\n", errno);
        return -errno;
    }

    /* Vòng lặp chính để đọc dữ liệu */
    while (1)
    {
        /* Đọc dữ liệu từ MPU6050 */
        ret = read(fd, &data, sizeof(struct mpu6050_data_s));
        if (ret != sizeof(struct mpu6050_data_s))
        {
            printf("Lỗi đọc dữ liệu từ MPU6050: %d\n", errno);
            close(fd);
            return -errno;
        }

        /* Tính góc nghiêng (roll và pitch) từ dữ liệu gia tốc */
        roll = atan2(data.accel_y, data.accel_z) * RAD_TO_DEG;
        pitch = atan2(-data.accel_x, sqrt(data.accel_y * data.accel_y + data.accel_z * data.accel_z)) * RAD_TO_DEG;

        /* In kết quả */
        printf("Roll: %.2f độ, Pitch: %.2f độ\n", roll, pitch);

        /* Chờ 500ms trước khi đọc lần tiếp theo */
        usleep(500000);
    }

    /* Đóng file descriptor */
    close(fd);
    return 0;
}