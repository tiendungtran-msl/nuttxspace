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

/* MPU6050 Registers */
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
static int mpu6050_write_reg(int fd, uint8_t reg, uint8_t data, uint8_t addr);
static int mpu6050_read_reg(int fd, uint8_t reg, uint8_t *data, uint8_t addr);
static int mpu6050_read_data(int fd, mpu6050_data_t *data, uint8_t addr);
static int mpu6050_init(int fd);
static void calculate_angles(mpu6050_data_t *raw_data, angle_data_t *angles, float dt);
static uint32_t get_time_ms(void);

/* Global I2C address */
static uint8_t g_mpu6050_addr = 0x68;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int mpu6050_write_reg(int fd, uint8_t reg, uint8_t data, uint8_t addr)
{
    struct i2c_transfer_s transfer;
    struct i2c_msg_s msg;
    uint8_t buffer[2];
    int ret;

    buffer[0] = reg;
    buffer[1] = data;

    msg.frequency = 400000;  /* 400kHz */
    msg.addr      = addr;
    msg.flags     = 0;
    msg.buffer    = buffer;
    msg.length    = 2;

    transfer.msgv = &msg;
    transfer.msgc = 1;

    ret = ioctl(fd, I2CIOC_TRANSFER, (unsigned long)&transfer);
    if (ret < 0)
    {
        syslog(LOG_ERR, "ERROR: I2C write failed (addr=0x%02X): %d (errno=%d)\n",
               addr, ret, errno);
        return ret;
    }

    return OK;
}

static int mpu6050_read_reg(int fd, uint8_t reg, uint8_t *data, uint8_t addr)
{
    struct i2c_transfer_s transfer;
    struct i2c_msg_s msg[2];
    int ret;

    if (data == NULL)
    {
        syslog(LOG_ERR, "ERROR: Null data pointer in mpu6050_read_reg\n");
        return -EINVAL;
    }

    msg[0].frequency = 400000;
    msg[0].addr      = addr;
    msg[0].flags     = 0;
    msg[0].buffer    = &reg;
    msg[0].length    = 1;

    msg[1].frequency = 400000;
    msg[1].addr      = addr;
    msg[1].flags     = I2C_M_READ | I2C_M_NOSTART;
    msg[1].buffer    = data;
    msg[1].length    = 1;

    transfer.msgv = msg;
    transfer.msgc = 2;

    ret = ioctl(fd, I2CIOC_TRANSFER, (unsigned long)&transfer);
    if (ret < 0)
    {
        syslog(LOG_ERR, "ERROR: I2C read failed (addr=0x%02X, reg=0x%02X): %d (errno=%d)\n",
               addr, reg, ret, errno);
        if (ret == -ETIMEDOUT)
        {
#ifdef CONFIG_I2C_RESET
            ret = ioctl(fd, I2CIOC_RESET, 0);
            if (ret < 0)
            {
                syslog(LOG_ERR, "ERROR: I2C reset failed: %d\n", ret);
            }
#endif
        }
        else if (ret == -EPERM || ret == -EIO)
        {
            syslog(LOG_INFO, "Retrying with 100kHz...\n");
            msg[0].frequency = 100000;
            msg[1].frequency = 100000;
            ret = ioctl(fd, I2CIOC_TRANSFER, (unsigned long)&transfer);
            if (ret < 0)
            {
                syslog(LOG_ERR, "ERROR: I2C read retry failed (addr=0x%02X, reg=0x%02X): %d (errno=%d)\n",
                       addr, reg, ret, errno);
            }
        }
        return ret;
    }

    return OK;
}

static int mpu6050_read_data(int fd, mpu6050_data_t *data, uint8_t addr)
{
    struct i2c_transfer_s transfer;
    struct i2c_msg_s msg[2];
    uint8_t reg = MPU6050_ACCEL_XOUT_H;
    uint8_t buffer[14];  /* 6 bytes accel + 2 bytes temp + 6 bytes gyro */
    int ret;

    if (data == NULL)
    {
        syslog(LOG_ERR, "ERROR: Null data pointer in mpu6050_read_data\n");
        return -EINVAL;
    }

    msg[0].frequency = 400000;
    msg[0].addr      = addr;
    msg[0].flags     = 0;
    msg[0].buffer    = &reg;
    msg[0].length    = 1;

    msg[1].frequency = 400000;
    msg[1].addr      = addr;
    msg[1].flags     = I2C_M_READ | I2C_M_NOSTART;
    msg[1].buffer    = buffer;
    msg[1].length    = 14;

    transfer.msgv = msg;
    transfer.msgc = 2;

    ret = ioctl(fd, I2CIOC_TRANSFER, (unsigned long)&transfer);
    if (ret < 0)
    {
        syslog(LOG_ERR, "ERROR: I2C read data failed (addr=0x%02X): %d (errno=%d)\n",
               addr, ret, errno);
        if (ret == -ETIMEDOUT)
        {
#ifdef CONFIG_I2C_RESET
            ret = ioctl(fd, I2CIOC_RESET, 0);
            if (ret < 0)
            {
                syslog(LOG_ERR, "ERROR: I2C reset failed: %d\n", ret);
            }
#endif
        }
        return ret;
    }

    data->accel_x = (int16_t)((buffer[0] << 8) | buffer[1]);
    data->accel_y = (int16_t)((buffer[2] << 8) | buffer[3]);
    data->accel_z = (int16_t)((buffer[4] << 8) | buffer[5]);
    data->gyro_x = (int16_t)((buffer[8] << 8) | buffer[9]);
    data->gyro_y = (int16_t)((buffer[10] << 8) | buffer[11]);
    data->gyro_z = (int16_t)((buffer[12] << 8) | buffer[13]);

    return OK;
}

static int mpu6050_init(int fd)
{
    int ret;
    uint8_t who_am_i;
    uint8_t addresses[] = {0x68, 0x69};
    int i;

    syslog(LOG_INFO, "Initializing MPU6050...\n");

    for (i = 0; i < 2; i++)
    {
        ret = mpu6050_read_reg(fd, MPU6050_WHO_AM_I, &who_am_i, addresses[i]);
        if (ret == OK && who_am_i == 0x68)
        {
            syslog(LOG_INFO, "MPU6050 detected at address 0x%02X\n", addresses[i]);
            g_mpu6050_addr = addresses[i];
            break;
        }
    }

    if (i == 2)
    {
        syslog(LOG_ERR, "ERROR: MPU6050 not detected (last WHO_AM_I=0x%02X)\n", who_am_i);
        return -ENODEV;
    }

    ret = mpu6050_write_reg(fd, MPU6050_PWR_MGMT_1, 0x00, g_mpu6050_addr);
    if (ret < 0)
    {
        syslog(LOG_ERR, "ERROR: Failed to wake MPU6050\n");
        return ret;
    }

    usleep(100000);

    ret = mpu6050_write_reg(fd, MPU6050_SMPLRT_DIV, 0x07, g_mpu6050_addr);
    if (ret < 0) return ret;

    ret = mpu6050_write_reg(fd, MPU6050_ACCEL_CONFIG, 0x00, g_mpu6050_addr);
    if (ret < 0) return ret;

    ret = mpu6050_write_reg(fd, MPU6050_GYRO_CONFIG, 0x00, g_mpu6050_addr);
    if (ret < 0) return ret;

    ret = mpu6050_write_reg(fd, MPU6050_CONFIG, 0x03, g_mpu6050_addr);
    if (ret < 0) return ret;

    syslog(LOG_INFO, "MPU6050 initialized successfully!\n");
    return OK;
}

static uint32_t get_time_ms(void)
{
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts) < 0)
    {
        syslog(LOG_ERR, "ERROR: Failed to get time\n");
        return 0;
    }
    return (uint32_t)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

static void calculate_angles(mpu6050_data_t *raw_data, angle_data_t *angles, float dt)
{
    if (dt <= 0 || dt > 1.0) dt = 0.05;

    float accel_x = raw_data->accel_x / ACCEL_SCALE_FACTOR;
    float accel_y = raw_data->accel_y / ACCEL_SCALE_FACTOR;
    float accel_z = raw_data->accel_z / ACCEL_SCALE_FACTOR;
    float gyro_x = raw_data->gyro_x / GYRO_SCALE_FACTOR;
    float gyro_y = raw_data->gyro_y / GYRO_SCALE_FACTOR;

    float accel_pitch = atan2(accel_y, sqrt(accel_x * accel_x + accel_z * accel_z)) * RAD_TO_DEG;
    float accel_roll = atan2(-accel_x, sqrt(accel_y * accel_y + accel_z * accel_z)) * RAD_TO_DEG;

    angles->gyro_pitch += gyro_x * dt;
    angles->gyro_roll += gyro_y * dt;

    float alpha = 0.95;
    angles->pitch = alpha * (angles->pitch + gyro_x * dt) + (1.0 - alpha) * accel_pitch;
    angles->roll = alpha * (angles->roll + gyro_y * dt) + (1.0 - alpha) * accel_roll;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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

    fd = open(I2C_DEVICE_PATH, O_RDWR);
    if (fd < 0)
    {
        syslog(LOG_ERR, "ERROR: Failed to open I2C device %s: %d\n", I2C_DEVICE_PATH, errno);
        return EXIT_FAILURE;
    }

    ret = mpu6050_init(fd);
    if (ret < 0)
    {
        syslog(LOG_ERR, "ERROR: Failed to initialize MPU6050\n");
        close(fd);
        return EXIT_FAILURE;
    }

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

    while (1)
    {
        ret = mpu6050_read_data(fd, &raw_data, g_mpu6050_addr);
        if (ret < 0)
        {
            syslog(LOG_ERR, "ERROR: Failed to read MPU6050 data\n");
            break;
        }

        current_time = get_time_ms();
        if (current_time == 0)
        {
            syslog(LOG_ERR, "ERROR: Failed to get time\n");
            break;
        }
        dt = (current_time - last_time) / 1000.0;
        last_time = current_time;

        calculate_angles(&raw_data, &angles, dt);

        syslog(LOG_INFO, "%lu\t\t%.2f\t\t%.2f\t\t%d\t%d\t%d\n",
               (unsigned long)current_time, angles.pitch, angles.roll,
               raw_data.accel_x, raw_data.accel_y, raw_data.accel_z);

        usleep(50000);
    }

    close(fd);
    syslog(LOG_INFO, "\nApplication terminated.\n");

    return EXIT_SUCCESS;
}