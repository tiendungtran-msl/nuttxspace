#include <nuttx/config.h>
#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <nuttx/i2c/i2c_master.h>
#include <math.h>

#define MPU6050_ADDR        0x68
#define MPU6050_WHO_AM_I    0x75
#define MPU6050_PWR_MGMT_1  0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B

/* Cấu trúc lưu dữ liệu MPU6050 */
typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
} mpu_data_t;

/* Hàm đọc thanh ghi */
static int mpu6050_read_reg(int fd, uint8_t reg, uint8_t *data)
{
    struct i2c_msg_s msg[2];
    uint8_t reg_addr = reg;
    int ret;

    /* Gửi địa chỉ thanh ghi */
    msg[0].frequency = 400000;
    msg[0].addr = MPU6050_ADDR;
    msg[0].flags = 0;
    msg[0].buffer = &reg_addr;
    msg[0].length = 1;

    /* Đọc dữ liệu */
    msg[1].frequency = 400000;
    msg[1].addr = MPU6050_ADDR;
    msg[1].flags = I2C_M_READ;
    msg[1].buffer = data;
    msg[1].length = 1;

    ret = I2C_TRANSFER(fd, msg, 2);
    return ret;
}

/* Hàm ghi thanh ghi */
static int mpu6050_write_reg(int fd, uint8_t reg, uint8_t value)
{
    struct i2c_msg_s msg;
    uint8_t buf[2];
    int ret;

    buf[0] = reg;
    buf[1] = value;

    msg.frequency = 400000;
    msg.addr = MPU6050_ADDR;
    msg.flags = 0;
    msg.buffer = buf;
    msg.length = 2;

    ret = I2C_TRANSFER(fd, &msg, 1);
    return ret;
}

/* Hàm đọc dữ liệu gia tốc */
static int read_accel_data(int fd, mpu_data_t *data)
{
    uint8_t buffer[6];
    uint8_t reg = MPU6050_ACCEL_XOUT_H;
    struct i2c_msg_s msg[2];
    int ret;

    msg[0].frequency = 400000;
    msg[0].addr = MPU6050_ADDR;
    msg[0].flags = 0;
    msg[0].buffer = &reg;
    msg[0].length = 1;

    msg[1].frequency = 400000;
    msg[1].addr = MPU6050_ADDR;
    msg[1].flags = I2C_M_READ;
    msg[1].buffer = buffer;
    msg[1].length = 6;

    ret = I2C_TRANSFER(fd, msg, 2);
    if (ret < 0) return ret;

    data->accel_x = (int16_t)((buffer[0] << 8) | buffer[1]);
    data->accel_y = (int16_t)((buffer[2] << 8) | buffer[3]);
    data->accel_z = (int16_t)((buffer[4] << 8) | buffer[5]);

    return OK;
}

/* Hàm chính */
int main(int argc, FAR char *argv[])
{
    int fd;
    uint8_t who_am_i;
    mpu_data_t accel_data;
    float roll, pitch;

    printf("MPU6050 Angle Measurement App\n");
    printf("=============================\n");

    /* Mở I2C device */
    fd = open("/dev/i2c1", O_RDWR);
    if (fd < 0) {
        printf("ERROR: Failed to open I2C device\n");
        return -ENODEV;
    }

    /* Kiểm tra ID của MPU6050 */
    if (mpu6050_read_reg(fd, MPU6050_WHO_AM_I, &who_am_i) < 0 || who_am_i != 0x68) {
        printf("ERROR: MPU6050 not detected\n");
        close(fd);
        return -ENODEV;
    }

    /* Khởi động MPU6050 */
    mpu6050_write_reg(fd, MPU6050_PWR_MGMT_1, 0x00);
    usleep(100000);

    while(1) {
        if (read_accel_data(fd, &accel_data) == OK) {
            /* Tính góc nghiêng */
            roll = atan2(accel_data.accel_y, accel_data.accel_z) * 180/M_PI;
            pitch = atan2(-accel_data.accel_x, sqrt(accel_data.accel_y*accel_data.accel_y + 
                          accel_data.accel_z*accel_data.accel_z)) * 180/M_PI;

            printf("\rRoll: %.2f° Pitch: %.2f°    ", roll, pitch);
            fflush(stdout);
        }
        usleep(100000);  /* Đọc mỗi 100ms */
    }

    close(fd);
    return 0;
}