// apps/data_logger/include/tasks.h

#ifndef __APPS_DATA_LOGGER_TASKS_H
#define __APPS_DATA_LOGGER_TASKS_H

#include <pthread.h>

/* Task entry points */
void *sensor_task(void *arg);
void *filter_task(void *arg);
void *storage_task(void *arg);
void *communication_task(void *arg);
void *display_task(void *arg);
void *watchdog_task(void *arg);

/* Watchdog functions */
int watchdog_register(pthread_t task_id, const char *name);
void watchdog_kick(pthread_t task_id);

/* Hardware initialization (platform-specific) */
int hardware_initialize(void);
void hardware_cleanup(void);

#endif /* __APPS_DATA_LOGGER_TASKS_H */
