/****************************************************************************
 * FILE: task_mpu6050.c
 * DESCRIPTION: MPU6050 sensor reading task implementation
 ****************************************************************************/

/*
#include "stm32_multitask.h"
*/

/****************************************************************************
 * Name: i2c_write_reg
 *
 * Description:
 *   Write a single byte to I2C device register
 *
 ****************************************************************************/

static int i2c_write_reg(int fd, uint8_t reg, uint8_t value)
{
  struct i2c_msg_s msg;
  uint8_t buffer[2];

  buffer[0] = reg;
  buffer[1] = value;

  msg.frequency = MPU6050_I2C_FREQ;
  msg.addr = MPU6050_I2C_ADDR;
  msg.flags = 0;
  msg.buffer = buffer;
  msg.length = 2;

  return ioctl(fd, I2CIOC_TRANSFER, (unsigned long)((uintptr_t)&msg));
}

/****************************************************************************
 * Name: i2c_read_regs
 *
 * Description:
 *   Read multiple bytes from I2C device registers
 *
 ****************************************************************************/

static int i2c_read_regs(int fd, uint8_t reg, FAR uint8_t *buffer,
                         uint8_t length)
{
  struct i2c_msg_s msgs[2];

  /* Write register address */

  msgs[0].frequency = MPU6050_I2C_FREQ;
  msgs[0].addr = MPU6050_I2C_ADDR;
  msgs[0].flags = 0;
  msgs[0].buffer = &reg;
  msgs[0].length = 1;

  /* Read data */

  msgs[1].frequency = MPU6050_I2C_FREQ;
  msgs[1].addr = MPU6050_I2C_ADDR;
  msgs[1].flags = I2C_M_READ;
  msgs[1].buffer = buffer;
  msgs[1].length = length;

  return ioctl(fd, I2CIOC_TRANSFER, (unsigned long)((uintptr_t)msgs));
}

/****************************************************************************
 * Name: mpu6050_init
 *
 * Description:
 *   Initialize MPU6050 sensor
 *
 ****************************************************************************/

int mpu6050_init(int fd)
{
  int ret;

  /* Wake up MPU6050 (clear sleep bit) */

  ret = i2c_write_reg(fd, MPU6050_REG_PWR_MGMT_1, 0x00);
  if (ret < 0)
    {
      return ret;
    }

  /* Wait for sensor to stabilize */

  msleep(100);

  return OK;
}

/****************************************************************************
 * Name: mpu6050_read_data
 *
 * Description:
 *   Read sensor data from MPU6050
 *
 ****************************************************************************/

int mpu6050_read_data(int fd, FAR struct mpu6050_data_s *data)
{
  uint8_t buffer[14];
  int ret;

  /* Read all sensor data (accel + temp + gyro) */

  ret = i2c_read_regs(fd, MPU6050_REG_ACCEL_XOUT_H, buffer, 14);
  if (ret < 0)
    {
      return ret;
    }

  /* Parse accelerometer data */

  data->accel_x = (int16_t)(buffer[0] << 8 | buffer[1]);
  data->accel_y = (int16_t)(buffer[2] << 8 | buffer[3]);
  data->accel_z = (int16_t)(buffer[4] << 8 | buffer[5]);

  /* Parse gyroscope data (skip temperature bytes 6-7) */

  data->gyro_x = (int16_t)(buffer[8] << 8 | buffer[9]);
  data->gyro_y = (int16_t)(buffer[10] << 8 | buffer[11]);
  data->gyro_z = (int16_t)(buffer[12] << 8 | buffer[13]);

  /* Calculate pitch and roll angles from accelerometer */

  data->pitch = atan2f((float)data->accel_y,
                       sqrtf((float)(data->accel_x * data->accel_x +
                                     data->accel_z * data->accel_z)))
                * 180.0f / M_PI;

  data->roll = atan2f((float)(-data->accel_x),
                      (float)data->accel_z)
               * 180.0f / M_PI;

  return OK;
}

/****************************************************************************
 * Name: task_mpu6050_entry
 *
 * Description:
 *   MPU6050 sensor reading task
 *
 ****************************************************************************/

FAR void *task_mpu6050_entry(FAR void *arg)
{
  int fd;
  struct mpu6050_data_s sensor_data;
  int ret;

  printf("[MPU6050] Task starting...\n");

  /* Open I2C device */

  fd = open(DEVICE_I2C, O_RDWR);
  if (fd < 0)
    {
      fprintf(stderr, "[MPU6050] ERROR: Failed to open %s: %d\n",
              DEVICE_I2C, errno);
      return NULL;
    }

  printf("[MPU6050] Device opened successfully\n");

  /* Initialize MPU6050 */

  ret = mpu6050_init(fd);
  if (ret < 0)
    {
      fprintf(stderr, "[MPU6050] ERROR: Initialization failed: %d\n", ret);
      close(fd);
      return NULL;
    }

  printf("[MPU6050] Sensor initialized successfully\n");
  printf("[MPU6050] Reading period: %d ms\n", DELAY_MPU6050_MS);

  /* Main task loop */

  while (1)
    {
      /* Read sensor data */

      ret = mpu6050_read_data(fd, &sensor_data);
      if (ret < 0)
        {
          fprintf(stderr, "[MPU6050] ERROR: Read failed: %d\n", ret);
        }
      else
        {
          printf("[MPU6050] Pitch: %6.2f° | Roll: %6.2f° | "
                 "Gyro(X,Y,Z): %6d, %6d, %6d\n",
                 sensor_data.pitch,
                 sensor_data.roll,
                 sensor_data.gyro_x,
                 sensor_data.gyro_y,
                 sensor_data.gyro_z);
        }

      /* Sleep for specified period */

      msleep(DELAY_MPU6050_MS);
    }

  /* Should never reach here */

  close(fd);
  return NULL;
}

/****************************************************************************
 * Name: msleep
 *
 * Description:
 *   Sleep for specified milliseconds
 *
 ****************************************************************************/

void msleep(unsigned int ms)
{
  usleep(ms * 1000);
}