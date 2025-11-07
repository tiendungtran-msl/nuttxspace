// apps/data_logger/src/tasks/filter_task.c

#include <nuttx/config.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

#include "config.h"
#include "data_types.h"
#include "ipc.h"
#include "tasks.h"

#define FILTER_WINDOW_SIZE 10

struct filter_state {
    float values[FILTER_WINDOW_SIZE];
    int count;
    int index;
};

/*
* 
*/
static struct filter_state g_filters[4] = {0};  // One per sensor

/*
* Thêm mẫu vào bộ lọc trượt
* filter: Trạng thái bộ lọc
* value: Giá trị mẫu mới
* Cơ chế: Lưu giá trị vào mảng vòng, cập nhật chỉ số và số lượng mẫu
*/
static void filter_add_sample(struct filter_state *filter, float value)
{
    filter->values[filter->index] = value;
    filter->index = (filter->index + 1) % FILTER_WINDOW_SIZE;
    
    if (filter->count < FILTER_WINDOW_SIZE) {
        filter->count++;
    }
}

static void filter_compute(struct filter_state *filter,
                           struct filtered_data *output)
{
    if (filter->count == 0) return;
    
    float sum = 0.0f;
    float min = filter->values[0];
    float max = filter->values[0];
    
    for (int i = 0; i < filter->count; i++) {
        float val = filter->values[i];
        sum += val;
        if (val < min) min = val;
        if (val > max) max = val;
    }
    
    output->value = filter->values[(filter->index - 1 + FILTER_WINDOW_SIZE) 
                                    % FILTER_WINDOW_SIZE];
    output->min = min;
    output->max = max;
    output->avg = sum / filter->count;
    output->sample_count = filter->count;
}

void *filter_task(void *arg)
{
    struct sensor_data sensor_data;
    struct filtered_data filtered;
    
    printf("Filter task started (tid=%lu)\n",
           (unsigned long)pthread_self());
    
    watchdog_register(pthread_self(), "filter_task");
    
    while (1) {
        /* Receive sensor data (blocking) */
        ssize_t nbytes = mq_receive(g_sensor_to_filter_mq,
                                     (char *)&sensor_data,
                                     sizeof(sensor_data), NULL);
        if (nbytes != sizeof(sensor_data)) {
            fprintf(stderr, "Filter: Invalid data received\n");
            continue;
        }
        
        /* Add to filter */
        if (sensor_data.sensor_id < 4) {
            filter_add_sample(&g_filters[sensor_data.sensor_id],
                             sensor_data.value);
            
            /* Compute filtered values */
            filtered.timestamp = sensor_data.timestamp;
            filtered.sensor_id = sensor_data.sensor_id;
            filter_compute(&g_filters[sensor_data.sensor_id], &filtered);
            
            /* Send to storage task */
            mq_send(g_filter_to_storage_mq, (char *)&filtered,
                    sizeof(filtered), 0);
            
            /* Send to display task (non-blocking, lower priority) */
            struct mq_attr attr;
            mq_getattr(g_filter_to_display_mq, &attr);
            if (attr.mq_curmsgs < attr.mq_maxmsg) {
                mq_send(g_filter_to_display_mq, (char *)&filtered,
                        sizeof(filtered), 0);
            }
        }
        
        watchdog_kick(pthread_self());
    }
    
    return NULL;
}
