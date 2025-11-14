/****************************************************************************
 * apps/examples/imu_system/utils/data_queue.c
 *
 * Lock-free circular buffer implementation
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <string.h>
#include <errno.h>
#include <stdatomic.h>

#include "data_queue.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: data_queue_init
 ****************************************************************************/

int data_queue_init(data_queue_t *queue, void *buffer,
                    size_t element_size, size_t capacity)
{
  if (!queue || !buffer || element_size == 0 || capacity == 0 ||
      capacity > DATA_QUEUE_MAX_SIZE)
    {
      return -EINVAL;
    }

  queue->buffer = buffer;
  queue->element_size = element_size;
  queue->capacity = capacity;
  queue->head = 0;
  queue->tail = 0;

  return OK;
}

/****************************************************************************
 * Name: data_queue_push
 ****************************************************************************/

int data_queue_push(data_queue_t *queue, const void *data)
{
  size_t head;
  size_t next_head;
  size_t tail;

  if (!queue || !data)
    {
      return -EINVAL;
    }

  /* Read current positions */

  head = queue->head;
  tail = queue->tail;

  /* Calculate next head position */

  next_head = (head + 1) % queue->capacity;

  /* Check if queue is full */

  if (next_head == tail)
    {
      return -EAGAIN;
    }

  /* Copy data to buffer */

  memcpy((uint8_t *)queue->buffer + (head * queue->element_size),
         data, queue->element_size);

  /* Update head position (atomic) */

  queue->head = next_head;

  return OK;
}

/****************************************************************************
 * Name: data_queue_pop
 ****************************************************************************/

int data_queue_pop(data_queue_t *queue, void *data)
{
  size_t head;
  size_t tail;
  size_t next_tail;

  if (!queue || !data)
    {
      return -EINVAL;
    }

  /* Read current positions */

  head = queue->head;
  tail = queue->tail;

  /* Check if queue is empty */

  if (head == tail)
    {
      return -EAGAIN;
    }

  /* Copy data from buffer */

  memcpy(data, (uint8_t *)queue->buffer + (tail * queue->element_size),
         queue->element_size);

  /* Calculate next tail position */

  next_tail = (tail + 1) % queue->capacity;

  /* Update tail position (atomic) */

  queue->tail = next_tail;

  return OK;
}

/****************************************************************************
 * Name: data_queue_count
 ****************************************************************************/

size_t data_queue_count(const data_queue_t *queue)
{
  size_t head;
  size_t tail;

  if (!queue)
    {
      return 0;
    }

  head = queue->head;
  tail = queue->tail;

  if (head >= tail)
    {
      return head - tail;
    }
  else
    {
      return queue->capacity - tail + head;
    }
}

/****************************************************************************
 * Name: data_queue_is_full
 ****************************************************************************/

bool data_queue_is_full(const data_queue_t *queue)
{
  size_t head;
  size_t tail;
  size_t next_head;

  if (!queue)
    {
      return false;
    }

  head = queue->head;
  tail = queue->tail;
  next_head = (head + 1) % queue->capacity;

  return (next_head == tail);
}

/****************************************************************************
 * Name: data_queue_is_empty
 ****************************************************************************/

bool data_queue_is_empty(const data_queue_t *queue)
{
  if (!queue)
    {
      return true;
    }

  return (queue->head == queue->tail);
}
