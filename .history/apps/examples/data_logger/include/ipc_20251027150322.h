// apps/data_logger/include/ipc.h

#ifndef __APPS_DATA_LOGGER_IPC_H
#define __APPS_DATA_LOGGER_IPC_H

#include <mqueue.h>
#include <pthread.h>
#include "data_types.h"

/* Message queue descriptors */
extern mqd_t g_sensor_to_filter_mq;
extern mqd_t g_filter_to_storage_mq;
extern mqd_t g_filter_to_display_mq;

/* Mutexes */
extern pthread_mutex_t g_status_mutex;
extern struct system_status g_system_status;

/* Function prototypes */
int ipc_initialize(void);
void ipc_cleanup(void);

#endif /* __APPS_DATA_LOGGER_IPC_H */
