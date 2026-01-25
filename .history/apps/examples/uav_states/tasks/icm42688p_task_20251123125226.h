/****************************************************************************
 * apps/examples/uav_states/drivers/sensors/icm42688p/icm42688p_task.h
 *
 * ICM-42688-P Producer/Consumer Task Architecture
 * High-speed data acquisition with decoupled display/logging
 *
 * Author: Tien Dung Tran
 * Date: 2025-11-23
 *
 ****************************************************************************/

#ifndef __APPS_EXAMPLES_UAV_STATES_DRIVERS_SENSORS_ICM42688P_TASK_H
#define __APPS_EXAMPLES_UAV_STATES_DRIVERS_SENSORS_ICM42688P_TASK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>
#include <semaphore.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Task configuration */
#define ICM42688P_PRODUCER_PRIORITY       SCHED_PRIORITY_MAX - 10
#define ICM42688P_CONSUMER_PRIORITY       SCHED_PRIORITY_DEFAULT
#define ICM42688P_PRODUCER_STACK_SIZE     2048
#define ICM42688P_CONSUMER_STACK_SIZE     4096

/* Sampling configuration */
#define ICM42688P_SAMPLE_RATE_HZ          1000    /* Producer rate */
#define ICM42688P_DISPLAY_RATE_HZ         2       /* Consumer rate (500ms) */
#define ICM42688P_SAMPLE_PERIOD_US        (1000000 / ICM42688P_SAMPLE_RATE_HZ)
#define ICM42688P_DISPLAY_PERIOD_US       (1000000 / ICM42688P_DISPLAY_RATE_HZ)

/* Ring buffer configuration */
#define ICM42688P_RING_BUFFER_SIZE        2048    /* Power of 2 for efficiency */
#define ICM42688P_RING_BUFFER_MASK        (ICM42688P_RING_BUFFER_SIZE - 1)

/* Statistics window */
#define ICM42688P_STATS_WINDOW_SIZE       1000    /* 1 second at 1kHz */

/* Return codes */
#define ICM42688P_TASK_OK                 0
#define ICM42688P_TASK_ERR_INIT          -1
#define ICM42688P_TASK_ERR_START         -2
#define ICM42688P_TASK_ERR_STOP          -3
#define ICM42688P_TASK_ERR_FULL          -4
#define ICM42688P_TASK_ERR_EMPTY         -5

/****************************************************************************
 * Public Types
 ****************************************************************************/

/**
 * IMU Sample Structure
 * Optimized for minimal memory footprint and cache efficiency
 */
typedef struct
{
  uint64_t timestamp_us;      /* Microsecond timestamp */
  uint32_t sequence;          /* Sequence number for detecting drops */
  
  /* Raw sensor data (16-bit for efficiency) */
  int16_t accel_x;            /* Accelerometer X (raw) */
  int16_t accel_y;            /* Accelerometer Y (raw) */
  int16_t accel_z;            /* Accelerometer Z (raw) */
  int16_t gyro_x;             /* Gyroscope X (raw) */
  int16_t gyro_y;             /* Gyroscope Y (raw) */
  int16_t gyro_z;             /* Gyroscope Z (raw) */
  int16_t temperature;        /* Temperature (raw) */
  
  uint8_t flags;              /* Status flags */
} __attribute__((packed)) icm42688p_sample_t;

/**
 * Ring Buffer Structure
 * Lock-free single producer, single consumer
 */
typedef struct
{
  icm42688p_sample_t buffer[ICM42688P_RING_BUFFER_SIZE];
  
  volatile uint32_t head;     /* Write index (producer) */
  volatile uint32_t tail;     /* Read index (consumer) */
  
  /* Padding to prevent false sharing between CPU cores */
  uint8_t padding[64];
  
  /* Statistics */
  volatile uint32_t overruns;      /* Number of buffer overflows */
  volatile uint32_t underruns;     /* Number of buffer underflows */
  volatile uint32_t total_samples; /* Total samples written */
  
} icm42688p_ring_buffer_t;

/**
 * Statistics Structure
 * Computed by consumer task
 */
typedef struct
{
  /* Sample statistics */
  uint32_t samples_processed;
  uint32_t samples_dropped;
  float actual_sample_rate;
  
  /* Data statistics (converted to physical units) */
  float accel_x_mean;
  float accel_y_mean;
  float accel_z_mean;
  float accel_magnitude_mean;
  
  float gyro_x_mean;
  float gyro_y_mean;
  float gyro_z_mean;
  float gyro_magnitude_mean;
  
  float temperature_mean;
  
  /* Min/Max values */
  float accel_x_min, accel_x_max;
  float accel_y_min, accel_y_max;
  float accel_z_min, accel_z_max;
  
  float gyro_x_min, gyro_x_max;
  float gyro_y_min, gyro_y_max;
  float gyro_z_min, gyro_z_max;
  
  /* Timing */
  uint64_t window_start_us;
  uint64_t window_end_us;
  
} icm42688p_statistics_t;

/**
 * Task Manager Structure
 * Controls producer and consumer tasks
 */
typedef struct
{
  /* Device handle */
  void *device;               /* Pointer to icm42688p_dev_t */
  uint8_t device_id;
  
  /* Ring buffer */
  icm42688p_ring_buffer_t ring_buffer;
  
  /* Task handles */
  pthread_t producer_thread;
  pthread_t consumer_thread;
  
  /* Task control */
  volatile bool running;
  volatile bool producer_ready;
  volatile bool consumer_ready;
  
  /* Synchronization */
  sem_t data_available;       /* Semaphore for consumer wake-up */
  pthread_mutex_t stats_lock; /* Protects statistics */
  
  /* Current statistics */
  icm42688p_statistics_t stats;
  
  /* Configuration */
  uint32_t producer_period_us;
  uint32_t consumer_period_us;
  
  /* Calibration data (shared from device) */
  float accel_sensitivity;
  float gyro_sensitivity;
  float gyro_offset_x;
  float gyro_offset_y;
  float gyro_offset_z;
  
  /* Error tracking */
  uint32_t producer_errors;
  uint32_t consumer_errors;
  
} icm42688p_task_manager_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: icm42688p_task_init
 * 
 * Description:
 *   Initialize task manager and ring buffer
 * 
 * Input Parameters:
 *   manager   - Pointer to task manager structure
 *   device    - Pointer to ICM42688P device structure
 *   device_id - Device ID
 * 
 * Returned Value:
 *   ICM42688P_TASK_OK on success, negative error code on failure
 ****************************************************************************/
int icm42688p_task_init(icm42688p_task_manager_t *manager,
                        void *device, uint8_t device_id);

/****************************************************************************
 * Name: icm42688p_task_start
 * 
 * Description:
 *   Start producer and consumer tasks
 * 
 * Input Parameters:
 *   manager - Pointer to task manager structure
 * 
 * Returned Value:
 *   ICM42688P_TASK_OK on success, negative error code on failure
 ****************************************************************************/
int icm42688p_task_start(icm42688p_task_manager_t *manager);

/****************************************************************************
 * Name: icm42688p_task_stop
 * 
 * Description:
 *   Stop producer and consumer tasks gracefully
 * 
 * Input Parameters:
 *   manager - Pointer to task manager structure
 * 
 * Returned Value:
 *   ICM42688P_TASK_OK on success, negative error code on failure
 ****************************************************************************/
int icm42688p_task_stop(icm42688p_task_manager_t *manager);

/****************************************************************************
 * Name: icm42688p_task_get_stats
 * 
 * Description:
 *   Get current statistics (thread-safe)
 * 
 * Input Parameters:
 *   manager - Pointer to task manager structure
 *   stats   - Pointer to output statistics structure
 * 
 * Returned Value:
 *   ICM42688P_TASK_OK on success, negative error code on failure
 ****************************************************************************/
int icm42688p_task_get_stats(icm42688p_task_manager_t *manager,
                             icm42688p_statistics_t *stats);

/****************************************************************************
 * Name: icm42688p_task_print_stats
 * 
 * Description:
 *   Print current statistics to console
 * 
 * Input Parameters:
 *   manager - Pointer to task manager structure
 ****************************************************************************/
void icm42688p_task_print_stats(icm42688p_task_manager_t *manager);

/****************************************************************************
 * Name: icm42688p_ring_buffer_init
 * 
 * Description:
 *   Initialize ring buffer
 * 
 * Input Parameters:
 *   rb - Pointer to ring buffer structure
 ****************************************************************************/
void icm42688p_ring_buffer_init(icm42688p_ring_buffer_t *rb);

/****************************************************************************
 * Name: icm42688p_ring_buffer_push
 * 
 * Description:
 *   Push sample to ring buffer (producer side)
 *   Lock-free, single producer safe
 * 
 * Input Parameters:
 *   rb     - Pointer to ring buffer structure
 *   sample - Pointer to sample to push
 * 
 * Returned Value:
 *   ICM42688P_TASK_OK on success
 *   ICM42688P_TASK_ERR_FULL if buffer is full (oldest sample dropped)
 ****************************************************************************/
int icm42688p_ring_buffer_push(icm42688p_ring_buffer_t *rb,
                               const icm42688p_sample_t *sample);

/****************************************************************************
 * Name: icm42688p_ring_buffer_pop_batch
 * 
 * Description:
 *   Pop multiple samples from ring buffer (consumer side)
 *   Lock-free, single consumer safe
 * 
 * Input Parameters:
 *   rb         - Pointer to ring buffer structure
 *   samples    - Output buffer for samples
 *   max_count  - Maximum number of samples to pop
 * 
 * Returned Value:
 *   Number of samples actually popped (0 if empty)
 ****************************************************************************/
uint32_t icm42688p_ring_buffer_pop_batch(icm42688p_ring_buffer_t *rb,
                                         icm42688p_sample_t *samples,
                                         uint32_t max_count);

/****************************************************************************
 * Name: icm42688p_ring_buffer_count
 * 
 * Description:
 *   Get number of samples currently in buffer
 * 
 * Input Parameters:
 *   rb - Pointer to ring buffer structure
 * 
 * Returned Value:
 *   Number of samples in buffer
 ****************************************************************************/
static inline uint32_t icm42688p_ring_buffer_count(
    const icm42688p_ring_buffer_t *rb)
{
  return (rb->head - rb->tail) & ICM42688P_RING_BUFFER_MASK;
}

/****************************************************************************
 * Name: icm42688p_ring_buffer_is_empty
 * 
 * Description:
 *   Check if ring buffer is empty
 ****************************************************************************/
static inline bool icm42688p_ring_buffer_is_empty(
    const icm42688p_ring_buffer_t *rb)
{
  return rb->head == rb->tail;
}

/****************************************************************************
 * Name: icm42688p_ring_buffer_is_full
 * 
 * Description:
 *   Check if ring buffer is full
 ****************************************************************************/
static inline bool icm42688p_ring_buffer_is_full(
    const icm42688p_ring_buffer_t *rb)
{
  return ((rb->head + 1) & ICM42688P_RING_BUFFER_MASK) == rb->tail;
}

#ifdef __cplusplus
}
#endif

#endif /* __APPS_EXAMPLES_UAV_STATES_TASKS_ICM42688P_TASK_H */