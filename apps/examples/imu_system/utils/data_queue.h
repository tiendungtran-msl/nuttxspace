/****************************************************************************
 * apps/examples/imu_system/utils/data_queue.h
 *
 * Lock-free circular buffer for inter-task communication
 *
 ****************************************************************************/

#ifndef __APPS_EXAMPLES_IMU_SYSTEM_UTILS_DATA_QUEUE_H
#define __APPS_EXAMPLES_IMU_SYSTEM_UTILS_DATA_QUEUE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DATA_QUEUE_MAX_SIZE 128

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef struct
{
  void *buffer;           /* Pointer to data buffer */
  size_t element_size;    /* Size of each element */
  size_t capacity;        /* Maximum number of elements */
  volatile size_t head;   /* Write position */
  volatile size_t tail;   /* Read position */
} data_queue_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: data_queue_init
 *
 * Description:
 *   Initialize a data queue
 *
 * Input Parameters:
 *   queue        - Queue structure
 *   buffer       - Pre-allocated buffer for data
 *   element_size - Size of each element
 *   capacity     - Maximum number of elements
 *
 * Returned Value:
 *   0 on success, negative errno on failure
 *
 ****************************************************************************/

int data_queue_init(data_queue_t *queue, void *buffer,
                    size_t element_size, size_t capacity);

/****************************************************************************
 * Name: data_queue_push
 *
 * Description:
 *   Push an element to the queue (non-blocking)
 *
 * Input Parameters:
 *   queue - Queue structure
 *   data  - Pointer to data to push
 *
 * Returned Value:
 *   0 on success, -EAGAIN if queue is full
 *
 ****************************************************************************/

int data_queue_push(data_queue_t *queue, const void *data);

/****************************************************************************
 * Name: data_queue_pop
 *
 * Description:
 *   Pop an element from the queue (non-blocking)
 *
 * Input Parameters:
 *   queue - Queue structure
 *   data  - Pointer to buffer for popped data
 *
 * Returned Value:
 *   0 on success, -EAGAIN if queue is empty
 *
 ****************************************************************************/

int data_queue_pop(data_queue_t *queue, void *data);

/****************************************************************************
 * Name: data_queue_count
 *
 * Description:
 *   Get number of elements in queue
 *
 * Input Parameters:
 *   queue - Queue structure
 *
 * Returned Value:
 *   Number of elements in queue
 *
 ****************************************************************************/

size_t data_queue_count(const data_queue_t *queue);

/****************************************************************************
 * Name: data_queue_is_full
 *
 * Description:
 *   Check if queue is full
 *
 * Input Parameters:
 *   queue - Queue structure
 *
 * Returned Value:
 *   true if full, false otherwise
 *
 ****************************************************************************/

bool data_queue_is_full(const data_queue_t *queue);

/****************************************************************************
 * Name: data_queue_is_empty
 *
 * Description:
 *   Check if queue is empty
 *
 * Input Parameters:
 *   queue - Queue structure
 *
 * Returned Value:
 *   true if empty, false otherwise
 *
 ****************************************************************************/

bool data_queue_is_empty(const data_queue_t *queue);

#endif /* __APPS_EXAMPLES_IMU_SYSTEM_UTILS_DATA_QUEUE_H */
