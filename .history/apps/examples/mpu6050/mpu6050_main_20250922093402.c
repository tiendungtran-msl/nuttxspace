#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <math.h>
#include <nuttx/i2c/i2c_master.h>

/* MPU6050 I2C Address */
#define MPU6050_ADDR        0x68

/* MPU6050 Register Addresses */
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
#define RAD_TO_DEG         57.2958

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

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int mpu6050_write_reg(int fd, uint8_t reg, uint8_t data)
{
    struct i2c_msg_s msg[1];
    uint8_t buffer[2];
    int ret;

    buffer[0] = reg;
    buffer[1] = data;

    msg[0].frequency = 400000;  /* 400kHz */
    msg[0].addr      = MPU6050_ADDR;
    msg[0].flags     = 0;
    msg[0].buffer    = buffer;
    msg[0].length    = 2;

    ret = ioctl(fd, I2CIOC_TRANSFER, (unsigned long)msg);
    if (ret < 0)
    {
        printf("ERROR: I2C write failed: %d\n", ret);
        return ret;
    }

    return OK;
}

static int mpu6050_read_reg(int fd, uint8_t reg, uint8_t *data)
{
    struct i2c_msg_s msg[2];
    int ret;

    msg[0].frequency = 400000;  /* 400kHz */
    msg[0].addr      = MPU6050_ADDR;
    msg[0].flags     = 0;
    msg[0].buffer    = &reg;
    msg[0].length    = 1;

    msg[1].frequency = 400000;
    msg[1].addr      = MPU6050_ADDR;
    msg[1].flags     = I2C_M_READ;
    msg[1].buffer    = data;
    msg[1].length    = 1;

    ret = ioctl(fd, I2CIOC_TRANSFER, (unsigned long)msg);
    if (ret < 0)
    {
        printf("ERROR: I2C read failed: %d\n", ret);
        return ret;
    }

    return OK;
}

static int mpu6050_read_data(int fd, mpu6050_data_t *data)
{
    struct i2c_msg_s msg[2];
    uint8_t reg = MPU6050_ACCEL_XOUT_H;
    uint8_t buffer[14];  /* 6 bytes accel + 2 bytes temp + 6 bytes gyro */
    int ret;

    msg[0].frequency = 400000;
    msg[0].addr      = MPU6050_ADDR;
    msg[0].flags     = 0;
    msg[0].buffer    = &reg;
    msg[0].length    = 1;

    msg[1].frequency = 400000;
    msg[1].addr      = MPU6050_ADDR;
    msg[1].flags     = I2C_M_READ;
    msg[1].buffer    = buffer;
    msg[1].length    = 14;

    ret = ioctl(fd, I2CIOC_TRANSFER, (unsigned long)msg);
    if (ret < 0)
    {
        printf("ERROR: I2C read data failed: %d\n", ret);
        return ret;
    }

    /* Parse accelerometer data */
    data->accel_x = (int16_t)((buffer[0] << 8) | buffer[1]);
    data->accel_y = (int16_t)((buffer[2] << 8) | buffer[3]);
    data->accel_z = (int16_t)((buffer[4] << 8) | buffer[5]);

    /* Parse gyroscope data (skip temperature bytes 6,7) */
    data->gyro_x = (int16_t)((buffer[8] << 8) | buffer[9]);
    data->gyro_y = (int16_t)((buffer[10] << 8) | buffer[11]);
    data->gyro_z = (int16_t)((buffer[12] << 8) | buffer[13]);

    return OK;
}

static int mpu6050_init(int fd)
{
    int ret;

    printf("Initializing MPU6050...\n");

    /* Wake up the MPU6050 */
    ret = mpu6050_write_reg(fd, MPU6050_PWR_MGMT_1, 0x00);
    if (ret < 0) return ret;

    usleep(100000); /* Wait 100ms */

    /* Set sample rate to 1kHz */
    ret = mpu6050_write_reg(fd, MPU6050_SMPLRT_DIV, 0x07);
    if (ret < 0) return ret;

    /* Configure accelerometer (±2g) */
    ret = mpu6050_write_reg(fd, MPU6050_ACCEL_CONFIG, 0x00);
    if (ret < 0) return ret;

    /* Configure gyroscope (±250°/s) */
    ret = mpu6050_write_reg(fd, MPU6050_GYRO_CONFIG, 0x00);
    if (ret < 0) return ret;

    /* Set digital low pass filter */
    ret = mpu6050_write_reg(fd, MPU6050_CONFIG, 0x03);
    if (ret < 0) return ret;

    printf("MPU6050 initialized successfully!\n");
    return OK;
}

static uint32_t get_time_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

static void calculate_angles(mpu6050_data_t *raw_data, angle_data_t *angles, float dt)
{
    /* Convert raw data to physical units */
    float accel_x = raw_data->accel_x / ACCEL_SCALE_FACTOR;
    float accel_y = raw_data->accel_y / ACCEL_SCALE_FACTOR;
    float accel_z = raw_data->accel_z / ACCEL_SCALE_FACTOR;
    
    float gyro_x = raw_data->gyro_x / GYRO_SCALE_FACTOR;
    float gyro_y = raw_data->gyro_y / GYRO_SCALE_FACTOR;

    /* Calculate angles from accelerometer */
    float accel_pitch = atan2(accel_y, sqrt(accel_x*accel_x + accel_z*accel_z)) * RAD_TO_DEG;
    float accel_roll = atan2(-accel_x, sqrt(accel_y*accel_y + accel_z*accel_z)) * RAD_TO_DEG;

    /* Integrate gyroscope data */
    angles->gyro_pitch += gyro_x * dt;
    angles->gyro_roll += gyro_y * dt;

    /* Complementary filter (95% gyro, 5% accel) */
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

    printf("MPU6050 Pitch/Roll Measurement App\n");
    printf("===================================\n");

    /* Open I2C device */
    fd = open(I2C_DEVICE_PATH, O_RDWR);
    if (fd < 0)
    {
        printf("ERROR: Failed to open I2C device %s\n", I2C_DEVICE_PATH);
        return EXIT_FAILURE;
    }

    /* Initialize MPU6050 */
    ret = mpu6050_init(fd);
    if (ret < 0)
    {
        printf("ERROR: Failed to initialize MPU6050\n");
        close(fd);
        return EXIT_FAILURE;
    }

    /* Initialize timing */
    last_time = get_time_ms();

    printf("\nStarting measurements (Press Ctrl+C to stop):\n");
    printf("Time(ms)\tPitch(°)\tRoll(°)\tAccel_X\tAccel_Y\tAccel_Z\n");
    printf("---------------------------------------------------------------\n");

    /* Main measurement loop */
    while (1)
    {
        /* Read sensor data */
        ret = mpu6050_read_data(fd, &raw_data);
        if (ret < 0)
        {
            printf("ERROR: Failed to read MPU6050 data\n");
            break;
        }

        /* Calculate time difference */
        current_time = get_time_ms();
        dt = (current_time - last_time) / 1000.0; /* Convert to seconds */
        last_time = current_time;

        /* Calculate pitch and roll angles */
        calculate_angles(&raw_data, &angles, dt);

        /* Print results */
        printf("%u\t\t%.2f\t\t%.2f\t\t%d\t%d\t%d\n", 
               current_time,
               angles.pitch, 
               angles.roll,
               raw_data.accel_x,
               raw_data.accel_y, 
               raw_data.accel_z);

        /* Delay for next measurement */
        usleep(50000); /* 50ms = 20Hz update rate */
    }

    /* Cleanup */
    close(fd);
    printf("\nApplication terminated.\n");
    
    return EXIT_SUCCESS;
}#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <math.h>
#include <nuttx/i2c/i2c_master.h>

/* MPU6050 I2C Address */
#define MPU6050_ADDR        0x68

/* MPU6050 Register Addresses */
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
#define RAD_TO_DEG         57.2958

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

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int mpu6050_write_reg(int fd, uint8_t reg, uint8_t data)
{
    struct i2c_msg_s msg[1];
    uint8_t buffer[2];
    int ret;

    buffer[0] = reg;
    buffer[1] = data;

    msg[0].frequency = 400000;  /* 400kHz */
    msg[0].addr      = MPU6050_ADDR;
    msg[0].flags     = 0;
    msg[0].buffer    = buffer;
    msg[0].length    = 2;

    ret = ioctl(fd, I2CIOC_TRANSFER, (unsigned long)msg);
    if (ret < 0)
    {
        printf("ERROR: I2C write failed: %d\n", ret);
        return ret;
    }

    return OK;
}

static int mpu6050_read_reg(int fd, uint8_t reg, uint8_t *data)
{
    struct i2c_msg_s msg[2];
    int ret;

    msg[0].frequency = 400000;  /* 400kHz */
    msg[0].addr      = MPU6050_ADDR;
    msg[0].flags     = 0;
    msg[0].buffer    = &reg;
    msg[0].length    = 1;

    msg[1].frequency = 400000;
    msg[1].addr      = MPU6050_ADDR;
    msg[1].flags     = I2C_M_READ;
    msg[1].buffer    = data;
    msg[1].length    = 1;

    ret = ioctl(fd, I2CIOC_TRANSFER, (unsigned long)msg);
    if (ret < 0)
    {
        printf("ERROR: I2C read failed: %d\n", ret);
        return ret;
    }

    return OK;
}

static int mpu6050_read_data(int fd, mpu6050_data_t *data)
{
    struct i2c_msg_s msg[2];
    uint8_t reg = MPU6050_ACCEL_XOUT_H;
    uint8_t buffer[14];  /* 6 bytes accel + 2 bytes temp + 6 bytes gyro */
    int ret;

    msg[0].frequency = 400000;
    msg[0].addr      = MPU6050_ADDR;
    msg[0].flags     = 0;
    msg[0].buffer    = &reg;
    msg[0].length    = 1;

    msg[1].frequency = 400000;
    msg[1].addr      = MPU6050_ADDR;
    msg[1].flags     = I2C_M_READ;
    msg[1].buffer    = buffer;
    msg[1].length    = 14;

    ret = ioctl(fd, I2CIOC_TRANSFER, (unsigned long)msg);
    if (ret < 0)
    {
        printf("ERROR: I2C read data failed: %d\n", ret);
        return ret;
    }

    /* Parse accelerometer data */
    data->accel_x = (int16_t)((buffer[0] << 8) | buffer[1]);
    data->accel_y = (int16_t)((buffer[2] << 8) | buffer[3]);
    data->accel_z = (int16_t)((buffer[4] << 8) | buffer[5]);

    /* Parse gyroscope data (skip temperature bytes 6,7) */
    data->gyro_x = (int16_t)((buffer[8] << 8) | buffer[9]);
    data->gyro_y = (int16_t)((buffer[10] << 8) | buffer[11]);
    data->gyro_z = (int16_t)((buffer[12] << 8) | buffer[13]);

    return OK;
}

static int mpu6050_init(int fd)
{
    int ret;

    printf("Initializing MPU6050...\n");

    /* Wake up the MPU6050 */
    ret = mpu6050_write_reg(fd, MPU6050_PWR_MGMT_1, 0x00);
    if (ret < 0) return ret;

    usleep(100000); /* Wait 100ms */

    /* Set sample rate to 1kHz */
    ret = mpu6050_write_reg(fd, MPU6050_SMPLRT_DIV, 0x07);
    if (ret < 0) return ret;

    /* Configure accelerometer (±2g) */
    ret = mpu6050_write_reg(fd, MPU6050_ACCEL_CONFIG, 0x00);
    if (ret < 0) return ret;

    /* Configure gyroscope (±250°/s) */
    ret = mpu6050_write_reg(fd, MPU6050_GYRO_CONFIG, 0x00);
    if (ret < 0) return ret;

    /* Set digital low pass filter */
    ret = mpu6050_write_reg(fd, MPU6050_CONFIG, 0x03);
    if (ret < 0) return ret;

    printf("MPU6050 initialized successfully!\n");
    return OK;
}

static uint32_t get_time_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

static void calculate_angles(mpu6050_data_t *raw_data, angle_data_t *angles, float dt)
{
    /* Convert raw data to physical units */
    float accel_x = raw_data->accel_x / ACCEL_SCALE_FACTOR;
    float accel_y = raw_data->accel_y / ACCEL_SCALE_FACTOR;
    float accel_z = raw_data->accel_z / ACCEL_SCALE_FACTOR;
    
    float gyro_x = raw_data->gyro_x / GYRO_SCALE_FACTOR;
    float gyro_y = raw_data->gyro_y / GYRO_SCALE_FACTOR;

    /* Calculate angles from accelerometer */
    float accel_pitch = atan2(accel_y, sqrt(accel_x*accel_x + accel_z*accel_z)) * RAD_TO_DEG;
    float accel_roll = atan2(-accel_x, sqrt(accel_y*accel_y + accel_z*accel_z)) * RAD_TO_DEG;

    /* Integrate gyroscope data */
    angles->gyro_pitch += gyro_x * dt;
    angles->gyro_roll += gyro_y * dt;

    /* Complementary filter (95% gyro, 5% accel) */
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

    printf("MPU6050 Pitch/Roll Measurement App\n");
    printf("===================================\n");

    /* Open I2C device */
    fd = open(I2C_DEVICE_PATH, O_RDWR);
    if (fd < 0)
    {
        printf("ERROR: Failed to open I2C device %s\n", I2C_DEVICE_PATH);
        return EXIT_FAILURE;
    }

    /* Initialize MPU6050 */
    ret = mpu6050_init(fd);
    if (ret < 0)
    {
        printf("ERROR: Failed to initialize MPU6050\n");
        close(fd);
        return EXIT_FAILURE;
    }

    /* Initialize timing */
    last_time = get_time_ms();

    printf("\nStarting measurements (Press Ctrl+C to stop):\n");
    printf("Time(ms)\tPitch(°)\tRoll(°)\tAccel_X\tAccel_Y\tAccel_Z\n");
    printf("---------------------------------------------------------------\n");

    /* Main measurement loop */
    while (1)
    {
        /* Read sensor data */
        ret = mpu6050_read_data(fd, &raw_data);
        if (ret < 0)
        {
            printf("ERROR: Failed to read MPU6050 data\n");
            break;
        }

        /* Calculate time difference */
        current_time = get_time_ms();
        dt = (current_time - last_time) / 1000.0; /* Convert to seconds */
        last_time = current_time;

        /* Calculate pitch and roll angles */
        calculate_angles(&raw_data, &angles, dt);

        /* Print results */
        printf("%u\t\t%.2f\t\t%.2f\t\t%d\t%d\t%d\n", 
               current_time,
               angles.pitch, 
               angles.roll,
               raw_data.accel_x,
               raw_data.accel_y, 
               raw_data.accel_z);

        /* Delay for next measurement */
        usleep(50000); /* 50ms = 20Hz update rate */
    }

    /* Cleanup */
    close(fd);
    printf("\nApplication terminated.\n");
    
    return EXIT_SUCCESS;
}