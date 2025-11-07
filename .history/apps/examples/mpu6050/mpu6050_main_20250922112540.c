#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <time.h>
#include <math.h>
#include <errno.h>
#include <nuttx/i2c/i2c_master.h>
#include <syslog.h>

#if !defined(CONFIG_I2C) || !defined(CONFIG_I2C_DRIVER)
#error "CONFIG_I2C and CONFIG_I2C_DRIVER must be enabled"
#endif

#if !defined(CONFIG_SYSLOG)
#error "CONFIG_SYSLOG must be enabled for logging"
#endif

/* MPU6050 I2C Address and Registers */
#define MPU6050_ADDR        0x68
#define MPU6050_WHO_AM_I    0x75
#define MPU6050_PWR_MGMT_1  0x6B
#define MPU6050_SMPLRT_DIV  0x19
#define MPU6050_CONFIG      0x1A
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_ACCEL_XOUT_L 0x3C
#define MPU6050_ACCEL_YOUT_H 0x3D
#define MPU6050_ACCEL_YOUT_L 0x3E
#define MPU6050_ACCEL_ZOUT_H 0x3F
#define MPU6050_ACCEL_ZOUT_L 0x40
#define MPU6050_GYRO_XOUT_H  0x43
#define MPU6050_GYRO_XOUT_L  0x44
#define MPU6050_GYRO_YOUT_H  0x45
#define MPU6050_GYRO_YOUT_L  0x46
#define MPU6050_GYRO_ZOUT_H  0x47
#define MPU6050_GYRO_ZOUT_L  0x48

/* I2C Device Path */
#define I2C_DEVICE_PATH "/dev/i2c1"

/* Conversion factors */
#define ACCEL_SCALE_FACTOR  16384.0  /* For ±2g range */
#define GYRO_SCALE_FACTOR   131.0    /* For ±250°/s range */
#define RAD_TO_DEG          57.2958

typedef struct {
    int16_t accel_x, accel_y, accel_z;
    int16_t gyro_x, gyro_y, gyro_z;
} mpu6050_data_t;

typedef struct {
    float pitch, roll;
    float gyro_pitch, gyro_roll;
    uint32_t last_time;
} angle_data_t;

/* Function prototypes */
static int mpu6050_write_reg(int fd, uint8_t reg, uint8_t data);
static int mpu6050_read_reg(int fd, uint8_t reg, uint8_t *data);
static int mpu6050_read_data(int fd, mpu6050_data_t *data);
static int mpu6050_init(int fd);
static void calculate_angles(mpu6050_data_t *raw_data, angle_data_t *angles, float dt);
static uint32_t get_time_ms(void);

/* ... (Giữ nguyên các hàm private như bạn đã cung cấp) ... */

int main(int argc, FAR char *argv[])
{
    int fd;
    int ret;
    mpu6050_data_t raw_data;
    angle_data_t angles = {0};
    uint32_t current_time, last_time;
    float dt;

    syslog(LOG_INFO, "MPU6050 Pitch/Roll Measurement App\n");
    syslog(LOG_INFO, "===================================\n");

    /* Open I2C device */
    fd = open(I2C_DEVICE_PATH, O_RDWR);
    if (fd < 0)
    {
        syslog(LOG_ERR, "ERROR: Failed to open I2C device %s: %d\n", I2C_DEVICE_PATH, errno);
        return EXIT_FAILURE;
    }

    /* Initialize MPU6050 */
    ret = mpu6050_init(fd);
    if (ret < 0)
    {
        syslog(LOG_ERR, "ERROR: Failed to initialize MPU6050\n");
        close(fd);
        return EXIT_FAILURE;
    }

    /* Initialize timing */
    last_time = get_time_ms();
    if (last_time == 0)
    {
        syslog(LOG_ERR, "ERROR: Failed to initialize timing\n");
        close(fd);
        return EXIT_FAILURE;
    }

    syslog(LOG_INFO, "\nStarting measurements (Press Ctrl+C to stop):\n");
    syslog(LOG_INFO, "Time(ms)\tPitch(°)\tRoll(°)\tAccel_X\tAccel_Y\tAccel_Z\n");
    syslog(LOG_INFO, "---------------------------------------------------------------\n");

    /* Main measurement loop */
    while (1)
    {
        /* Read sensor data */
        ret = mpu6050_read_data(fd, &raw_data);
        if (ret < 0)
        {
            syslog(LOG_ERR, "ERROR: Failed to read MPU6050 data\n");
            break;
        }

        /* Calculate time difference */
        current_time = get_time_ms();
        if (current_time == 0)
        {
            syslog(LOG_ERR, "ERROR: Failed to get time\n");
            break;
        }
        dt = (current_time - last_time) / 1000.0; /* Convert to seconds */
        last_time = current_time;

        /* Calculate pitch and roll angles */
        calculate_angles(&raw_data, &angles, dt);

        /* Print results */
        syslog(LOG_INFO, "%lu\t\t%.2f\t\t%.2f\t\t%d\t%d\t%d\n",
               (unsigned long)current_time, angles.pitch, angles.roll,
               raw_data.accel_x, raw_data.accel_y, raw_data.accel_z);

        /* Delay for next measurement */
        usleep(50000); /* 50ms = 20Hz update rate */
    }

    /* Cleanup */
    close(fd);
    syslog(LOG_INFO, "\nApplication terminated.\n");

    return EXIT_SUCCESS;
}