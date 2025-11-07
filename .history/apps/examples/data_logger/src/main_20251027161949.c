// apps/data_logger/src/main.c (Complete Version)

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <sched.h>
#include <pthread.h>
#include <signal.h>
#include <errno.h>

#include "config.h"
#include "tasks.h"
#include "ipc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define APP_NAME "Data Logger"
#define APP_VERSION "1.0.0"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct {
    bool initialized;
    bool running;
    
    pthread_t watchdog_thread;
    pthread_t sensor_thread;
    pthread_t filter_thread;
    pthread_t storage_thread;
    pthread_t comm_thread;
    pthread_t display_thread;
} g_app;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void signal_handler(int signo)
{
    printf("\n%s: Received signal %d, shutting down...\n", 
           APP_NAME, signo);
    g_app.running = false;
}

static int setup_signal_handlers(void)
{
    struct sigaction sa;
    
    sa.sa_handler = signal_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    
    if (sigaction(SIGINT, &sa, NULL) < 0) {
        fprintf(stderr, "ERROR: Failed to setup SIGINT handler\n");
        return -errno;
    }
    
    if (sigaction(SIGTERM, &sa, NULL) < 0) {
        fprintf(stderr, "ERROR: Failed to setup SIGTERM handler\n");
        return -errno;
    }
    
    return OK;
}

static int app_initialize(void)
{
    int ret;
    
    printf("======================================\n");
    printf(" %s v%s\n", APP_NAME, APP_VERSION);
    printf("======================================\n");
    printf("Initializing...\n");
    
    /* Setup signal handlers */
    ret = setup_signal_handlers();
    if (ret < 0) {
        fprintf(stderr, "ERROR: Signal handler setup failed\n");
        return ret;
    }
    
    /* Initialize IPC mechanisms */
    ret = ipc_initialize();
    if (ret < 0) {
        fprintf(stderr, "ERROR: IPC initialization failed: %d\n", ret);
        return ret;
    }
    
    /* Initialize hardware */
    ret = hardware_initialize();
    if (ret < 0) {
        fprintf(stderr, "ERROR: Hardware initialization failed: %d\n", ret);
        return ret;
    }
    
    g_app.initialized = true;
    printf("Initialization complete\n\n");
    
    return OK;
}

static int start_task(pthread_t *thread, void *(*start_routine)(void *),
                      int priority, int stack_size, const char *name)
{
    pthread_attr_t attr;
    struct sched_param param;
    int ret;
    
    pthread_attr_init(&attr);
    pthread_attr_setstacksize(&attr, stack_size);
    
    param.sched_priority = priority;
    pthread_attr_setschedparam(&attr, &param);
    
    ret = pthread_create(thread, &attr, start_routine, NULL);
    pthread_attr_destroy(&attr);
    
    if (ret != 0) {
        fprintf(stderr, "ERROR: Failed to create %s: %d\n", name, ret);
        return -ret;
    }
    
    printf("Started: %-20s (priority=%3d, stack=%d, tid=%lu)\n",
           name, priority, stack_size, (unsigned long)*thread);
    
    return OK;
}

static int app_start_tasks(void)
{
    int ret;
    
    printf("Starting tasks...\n");
    
    /* Start watchdog task (highest priority) */
    ret = start_task(&g_app.watchdog_thread, watchdog_task,
                     PRIO_WATCHDOG, STACK_WATCHDOG,
                     "Watchdog Task");
    if (ret < 0) return ret;
    
    /* Start sensor task (high priority) */
    ret = start_task(&g_app.sensor_thread, sensor_task,
                     PRIO_SENSOR, STACK_SENSOR,
                     "Sensor Task");
    if (ret < 0) return ret;
    
    /* Start filter task (medium-high priority) */
    ret = start_task(&g_app.filter_thread, filter_task,
                     PRIO_FILTER, STACK_FILTER,
                     "Filter Task");
    if (ret < 0) return ret;
    
    /* Start storage task (medium priority) */
    ret = start_task(&g_app.storage_thread, storage_task,
                     PRIO_STORAGE, STACK_STORAGE,
                     "Storage Task");
    if (ret < 0) return ret;
    
    /* Start display task (medium priority) */
    ret = start_task(&g_app.display_thread, display_task,
                     PRIO_DISPLAY, STACK_DISPLAY,
                     "Display Task");
    if (ret < 0) return ret;
    
    /* Start communication task (low priority) */
    ret = start_task(&g_app.comm_thread, communication_task,
                     PRIO_COMM, STACK_COMM,
                     "Communication Task");
    if (ret < 0) return ret;
    
    printf("\nAll tasks started successfully!\n\n");
    
    return OK;
}

static void app_stop_tasks(void)
{
    printf("\nStopping tasks...\n");
    
    /* Cancel all threads */
    pthread_cancel(g_app.comm_thread);
    pthread_cancel(g_app.display_thread);
    pthread_cancel(g_app.storage_thread);
    pthread_cancel(g_app.filter_thread);
    pthread_cancel(g_app.sensor_thread);
    pthread_cancel(g_app.watchdog_thread);
    
    /* Wait for threads to finish */
    pthread_join(g_app.comm_thread, NULL);
    pthread_join(g_app.display_thread, NULL);
    pthread_join(g_app.storage_thread, NULL);
    pthread_join(g_app.filter_thread, NULL);
    pthread_join(g_app.sensor_thread, NULL);
    pthread_join(g_app.watchdog_thread, NULL);
    
    printf("All tasks stopped\n");
}

static void app_cleanup(void)
{
    printf("\nCleaning up...\n");
    
    /* Cleanup IPC */
    ipc_cleanup();
    
    /* Cleanup hardware */
    hardware_cleanup();
    
    printf("Cleanup complete\n");
}

static void print_status_line(void)
{
    struct system_status status;
    
    pthread_mutex_lock(&g_status_mutex);
    status = g_system_status;
    pthread_mutex_unlock(&g_status_mutex);
    
    printf("\r[%c%c%c] Samples: %-8u Storage: %-6u KB  ",
           status.sensors_ok ? 'S' : '-',
           status.storage_ok ? 'D' : '-',
           status.comm_ok ? 'C' : '-',
           status.total_samples,
           status.storage_used_kb);
    fflush(stdout);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
    int ret;
    
    /* Initialize application */
    ret = app_initialize();
    if (ret < 0) {
        fprintf(stderr, "ERROR: Application initialization failed\n");
        return EXIT_FAILURE;
    }
    
    /* Start all tasks */
    ret = app_start_tasks();
    if (ret < 0) {
        fprintf(stderr, "ERROR: Failed to start tasks\n");
        app_cleanup();
        return EXIT_FAILURE;
    }
    
    /* Set running flag */
    g_app.running = true;
    
    printf("System running... (Press Ctrl+C to exit)\n");
    printf("Status: [S]ensor [D]isk [C]omm\n\n");
    
    /* Main loop - monitor system */
    while (g_app.running) {
        print_status_line();
        sleep(1);
    }
    
    printf("\n\nShutting down...\n");
    
    /* Stop all tasks */
    app_stop_tasks();
    
    /* Cleanup */
    app_cleanup();
    
    printf("\n%s terminated\n", APP_NAME);
    
    return EXIT_SUCCESS;
}
