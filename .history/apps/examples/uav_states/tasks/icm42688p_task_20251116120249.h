/****************************************************************************
 * apps/examples/uav_states/tasks/icm42688p_task.h
 ****************************************************************************/

#ifndef __APPS_EXAMPLES_UAV_STATES_TASKS_ICM42688P_TASK_H
#define __APPS_EXAMPLES_UAV_STATES_TASKS_ICM42688P_TASK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdbool.h>
#include "../drivers/sensors/icm42688p/icm42688p_driver.h"

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/* Start ICM42688P reading task */
int icm42688p_task_start(void);

/* Stop ICM42688P reading task */
int icm42688p_task_stop(void);

/* Get latest data (thread-safe) */
int icm42688p_task_get_data(icm42688p_data_t *data);

/* Check if task is running */
bool icm42688p_task_is_running(void);

/* Print task status */
void icm42688p_task_print_status(void);

#ifdef __cplusplus
}
#endif

#endif /* __APPS_EXAMPLES_UAV_STATES_TASKS_ICM42688P_TASK_H */