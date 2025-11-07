// apps/data_logger/src/tasks/watchdog_task.c

#include <nuttx/config.h>
#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>
#include <errno.h>

#include "config.h"
#include "tasks.h"

#define MAX_MONITORED_TASKS 8

struct task_watchdog {
    pthread_t task_id;
    const char *task_name;
    uint32_t last_kick_time;
    uint32_t timeout_ms;
    bool enabled;
    int timeout_count;
};

static struct task_watchdog g_watchdogs[MAX_MONITORED_TASKS];
static int g_num_watchdogs = 0;
static pthread_mutex_t g_watchdog_mutex = PTHREAD_MUTEX_INITIALIZER;

/* Get current time in milliseconds */
static uint32_t get_tick_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (ts.tv_sec * 1000) + (ts.tv_nsec / 1000000);
}

int watchdog_register(pthread_t task_id, const char *name)
{
    pthread_mutex_lock(&g_watchdog_mutex);
    
    if (g_num_watchdogs >= MAX_MONITORED_TASKS) {
        pthread_mutex_unlock(&g_watchdog_mutex);
        fprintf(stderr, "Watchdog: Too many tasks registered\n");
        return -ENOMEM;
    }
    
    g_watchdogs[g_num_watchdogs].task_id = task_id;
    g_watchdogs[g_num_watchdogs].task_name = name;
    g_watchdogs[g_num_watchdogs].last_kick_time = get_tick_ms();
    g_watchdogs[g_num_watchdogs].timeout_ms = 
        CONFIG_DATA_LOGGER_WATCHDOG_TIMEOUT;
    g_watchdogs[g_num_watchdogs].enabled = true;
    g_watchdogs[g_num_watchdogs].timeout_count = 0;
    g_num_watchdogs++;
    
    pthread_mutex_unlock(&g_watchdog_mutex);
    
    printf("Watchdog: Registered task '%s' (tid=%lu, timeout=%u ms)\n",
           name, (unsigned long)task_id,
           CONFIG_DATA_LOGGER_WATCHDOG_TIMEOUT);
    
    return OK;
}

void watchdog_kick(pthread_t task_id)
{
    pthread_mutex_lock(&g_watchdog_mutex);
    
    for (int i = 0; i < g_num_watchdogs; i++) {
        if (pthread_equal(g_watchdogs[i].task_id, task_id)) {
            g_watchdogs[i].last_kick_time = get_tick_ms();
            g_watchdogs[i].timeout_count = 0;  // Reset timeout counter
            break;
        }
    }
    
    pthread_mutex_unlock(&g_watchdog_mutex);
}

static void handle_task_timeout(struct task_watchdog *wd)
{
    fprintf(stderr, 
            "\n!!! WATCHDOG TIMEOUT !!!\n"
            "Task: %s (tid=%lu)\n"
            "Timeout: %u ms\n"
            "Timeout count: %d\n\n",
            wd->task_name,
            (unsigned long)wd->task_id,
            wd->timeout_ms,
            wd->timeout_count);
    
    /* Recovery actions */
    if (wd->timeout_count < 3) {
        /* Try to cancel and restart the task */
        printf("Watchdog: Attempting task recovery...\n");
        
        /* TODO: Implement task restart logic */
        /* pthread_cancel(wd->task_id); */
        /* restart_task(wd->task_name); */
        
    } else {
        /* Too many timeouts - system reset */
        fprintf(stderr, "Watchdog: Too many timeouts, system reset!\n");
        
        /* TODO: Implement system reset */
        /* boardctl(BOARDIOC_RESET, 0); */
    }
    
    wd->timeout_count++;
}

void *watchdog_task(void *arg)
{
    printf("Watchdog task started (tid=%lu, highest priority)\n",
           (unsigned long)pthread_self());
    
    while (1) {
        sleep(1);  // Check every second
        
        pthread_mutex_lock(&g_watchdog_mutex);
        
        uint32_t now = get_tick_ms();
        
        for (int i = 0; i < g_num_watchdogs; i++) {
            if (!g_watchdogs[i].enabled) continue;
            
            uint32_t elapsed = now - g_watchdogs[i].last_kick_time;
            
            if (elapsed > g_watchdogs[i].timeout_ms) {
                handle_task_timeout(&g_watchdogs[i]);
                
                /* Reset last kick time to avoid repeated triggers */
                g_watchdogs[i].last_kick_time = now;
            }
        }
        
        pthread_mutex_unlock(&g_watchdog_mutex);
    }
    
    return NULL;
}
