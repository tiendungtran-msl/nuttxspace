// apps/data_logger/include/config.h

#ifndef __APPS_DATA_LOGGER_CONFIG_H
#define __APPS_DATA_LOGGER_CONFIG_H

#include <nuttx/config.h>

/* Priority definitions */
#define PRIO_WATCHDOG  (SCHED_PRIORITY_MAX - 1)        // 254
#define PRIO_SENSOR    (SCHED_PRIORITY_DEFAULT + 50)  // 150
#define PRIO_FILTER    (SCHED_PRIORITY_DEFAULT + 20)  // 120
#define PRIO_STORAGE   (SCHED_PRIORITY_DEFAULT + 10)  // 110
#define PRIO_DISPLAY   (SCHED_PRIORITY_DEFAULT)       // 100
#define PRIO_COMM      (SCHED_PRIORITY_DEFAULT - 20)  // 80

/* Stack sizes */
#define STACK_WATCHDOG  2048
#define STACK_SENSOR    4096
#define STACK_FILTER    4096
#define STACK_STORAGE   4096
#define STACK_DISPLAY   2048
#define STACK_COMM      4096

/* Configuration */
#ifndef CONFIG_DATA_LOGGER_SENSOR_INTERVAL
#define CONFIG_DATA_LOGGER_SENSOR_INTERVAL 100
#endif

#ifndef CONFIG_DATA_LOGGER_STORAGE_INTERVAL
#define CONFIG_DATA_LOGGER_STORAGE_INTERVAL 1000
#endif

#endif /* __APPS_DATA_LOGGER_CONFIG_H */
