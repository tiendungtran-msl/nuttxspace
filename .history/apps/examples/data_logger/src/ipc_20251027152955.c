// apps/data_logger/src/ipc.c

#include <nuttx/config.h>
#include <stdio.h>
#include <errno.h>
#include <mqueue.h>
#include <pthread.h>

#include "ipc.h"
#include "data_types.h"

/* Message queues */
mqd_t g_sensor_to_filter_mq = (mqd_t)-1;
mqd_t g_filter_to_storage_mq = (mqd_t)-1;
mqd_t g_filter_to_display_mq = (mqd_t)-1;

/* Shared data */
pthread_mutex_t g_status_mutex = PTHREAD_MUTEX_INITIALIZER;
struct system_status g_system_status = {0};


int ipc_initialize(void)
{
    struct mq_attr attr;
    
    /* Sensor to Filter queue */
    attr.mq_maxmsg = 20;
    attr.mq_msgsize = sizeof(struct sensor_data);
    attr.mq_flags = 0; // Blocking mode
    
    g_sensor_to_filter_mq = mq_open("/sensor_filter", 
                                     O_CREAT | O_RDWR, 
                                     0644, &attr);
    if (g_sensor_to_filter_mq == (mqd_t)-1) {
        fprintf(stderr, "ERROR: Failed to create sensor->filter queue\n");
        return -errno;
    }
    
    /* Filter to Storage queue */
    attr.mq_maxmsg = 10;
    attr.mq_msgsize = sizeof(struct filtered_data);
    
    g_filter_to_storage_mq = mq_open("/filter_storage",
                                      O_CREAT | O_RDWR,
                                      0644, &attr);
    if (g_filter_to_storage_mq == (mqd_t)-1) {
        fprintf(stderr, "ERROR: Failed to create filter->storage queue\n");
        goto errout_sensor_filter;
    }
    
    /* Filter to Display queue */
    attr.mq_maxmsg = 5;
    attr.mq_msgsize = sizeof(struct filtered_data);
    
    g_filter_to_display_mq = mq_open("/filter_display",
                                      O_CREAT | O_RDWR,
                                      0644, &attr);
    if (g_filter_to_display_mq == (mqd_t)-1) {
        fprintf(stderr, "ERROR: Failed to create filter->display queue\n");
        goto errout_filter_storage;
    }
    
    /* Initialize system status */
    pthread_mutex_lock(&g_status_mutex);
    g_system_status.sensors_ok = false;
    g_system_status.storage_ok = false;
    g_system_status.comm_ok = false;
    g_system_status.total_samples = 0;
    g_system_status.storage_used_kb = 0;
    pthread_mutex_unlock(&g_status_mutex);
    
    printf("IPC initialized successfully\n");
    return OK;

errout_filter_storage:
    mq_close(g_filter_to_storage_mq);
    mq_unlink("/filter_storage");
errout_sensor_filter:
    mq_close(g_sensor_to_filter_mq);
    mq_unlink("/sensor_filter");
    return -errno;
}

void ipc_cleanup(void)
{
    /* Close and unlink message queues */
    if (g_sensor_to_filter_mq != (mqd_t)-1) {
        mq_close(g_sensor_to_filter_mq);
        mq_unlink("/sensor_filter");
    }
    
    if (g_filter_to_storage_mq != (mqd_t)-1) {
        mq_close(g_filter_to_storage_mq);
        mq_unlink("/filter_storage");
    }
    
    if (g_filter_to_display_mq != (mqd_t)-1) {
        mq_close(g_filter_to_display_mq);
        mq_unlink("/filter_display");
    }
    
    /* Destroy mutex */
    pthread_mutex_destroy(&g_status_mutex);
    
    printf("IPC cleaned up\n");
}
