// apps/data_logger/src/tasks/display_task.c

#include <nuttx/config.h>
#include <stdio.h>
#include <unistd.h>

#include "config.h"
#include "data_types.h"
#include "ipc.h"
#include "tasks.h"

/* LCD display functions (simulated - replace with real driver) */
static void lcd_clear(void)
{
    /* TODO: Clear LCD screen */
    printf("\033[2J\033[H");  // ANSI clear screen
}

static void lcd_print(int row, int col, const char *text)
{
    /* TODO: Print to LCD at position */
    printf("\033[%d;%dH%s", row + 1, col + 1, text);
}

void *display_task(void *arg)
{
    struct filtered_data data;
    struct system_status status;
    char line[32];
    struct timespec timeout;
    
    printf("Display task started (tid=%lu)\n",
           (unsigned long)pthread_self());
    
    watchdog_register(pthread_self(), "display_task");
    
    lcd_clear();
    
    while (1) {
        /* Try to receive new data (with timeout) */
        clock_gettime(CLOCK_REALTIME, &timeout);
        timeout.tv_sec += 1;  // 1 second timeout
        
        ssize_t nbytes = mq_timedreceive(g_filter_to_display_mq,
                                          (char *)&data,
                                          sizeof(data), NULL,
                                          &timeout);
        
        /* Get system status */
        pthread_mutex_lock(&g_status_mutex);
        status = g_system_status;
        pthread_mutex_unlock(&g_status_mutex);
        
        /* Update display */
        lcd_print(0, 0, "=== Data Logger ===");
        
        if (nbytes == sizeof(data)) {
            snprintf(line, sizeof(line), "S%u: %.2f (%.2f-%.2f)",
                    data.sensor_id, data.avg, data.min, data.max);
            lcd_print(2 + data.sensor_id, 0, line);
        }
        
        /* Display status */
        snprintf(line, sizeof(line), "Samples: %u",
                status.total_samples);
        lcd_print(6, 0, line);
        
        snprintf(line, sizeof(line), "Storage: %u KB",
                status.storage_used_kb);
        lcd_print(7, 0, line);
        
        snprintf(line, sizeof(line), "Status: %c%c%c",
                status.sensors_ok ? 'S' : '-',
                status.storage_ok ? 'D' : '-',
                status.comm_ok ? 'C' : '-');
        lcd_print(8, 0, line);
        
        watchdog_kick(pthread_self());
        
        usleep(CONFIG_DATA_LOGGER_DISPLAY_REFRESH * 1000);
    }
    
    return NULL;
}
