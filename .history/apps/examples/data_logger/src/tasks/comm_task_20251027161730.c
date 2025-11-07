// apps/data_logger/src/tasks/comm_task.c 

#include <nuttx/config.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>

#include "config.h"
#include "data_types.h"
#include "ipc.h"
#include "tasks.h"

#define UART_DEVICE "/dev/ttyS1"
#define COMM_BUFFER_SIZE 256

static int g_uart_fd = -1;

static int comm_open_uart(void)
{
    g_uart_fd = open(UART_DEVICE, O_WRONLY | O_NONBLOCK);
    if (g_uart_fd < 0) {
        fprintf(stderr, "Comm: Failed to open UART: %d\n", errno);
        return -errno;
    }
    
    /* TODO: Configure UART (baud rate, etc.) */
    
    printf("Comm: UART opened (%s)\n", UART_DEVICE);
    return OK;
}

static int comm_send_data(const char *data, size_t len)
{
    if (g_uart_fd < 0) return -EBADF;
    
    ssize_t written = write(g_uart_fd, data, len);
    if (written < 0) {
        return -errno;
    }
    
    return written;
}

void *communication_task(void *arg)
{
    char buffer[COMM_BUFFER_SIZE];
    struct filtered_data data;
    struct timespec timeout;
    int ret;
    
    printf("Communication task started (tid=%lu)\n",
           (unsigned long)pthread_self());
    
    watchdog_register(pthread_self(), "comm_task");
    
    /* Open UART */
    ret = comm_open_uart();
    if (ret < 0) {
        pthread_mutex_lock(&g_status_mutex);
        g_system_status.comm_ok = false;
        pthread_mutex_unlock(&g_status_mutex);
        return NULL;
    }
    
    pthread_mutex_lock(&g_status_mutex);
    g_system_status.comm_ok = true;
    pthread_mutex_unlock(&g_status_mutex);
    
    while (1) {
        /* Try to get data from storage queue (non-blocking peek) */
        clock_gettime(CLOCK_REALTIME, &timeout);
        timeout.tv_sec += 2;  // 2 second timeout
        
        ssize_t nbytes = mq_timedreceive(g_filter_to_storage_mq,
                                          (char *)&data,
                                          sizeof(data), NULL,
                                          &timeout);
        
        if (nbytes == sizeof(data)) {
            /* Format message for transmission */
            int len = snprintf(buffer, sizeof(buffer),
                              "$DATA,%u,%u,%.2f,%.2f,%.2f,%.2f\r\n",
                              data.timestamp,
                              data.sensor_id,
                              data.value,
                              data.min,
                              data.max,
                              data.avg);
            
            /* Send via UART */
            ret = comm_send_data(buffer, len);
            if (ret < 0) {
                fprintf(stderr, "Comm: Failed to send data: %d\n", ret);
                
                pthread_mutex_lock(&g_status_mutex);
                g_system_status.comm_ok = false;
                pthread_mutex_unlock(&g_status_mutex);
            } else {
                pthread_mutex_lock(&g_status_mutex);
                g_system_status.comm_ok = true;
                pthread_mutex_unlock(&g_status_mutex);
            }
        }
        
        watchdog_kick(pthread_self());
    }
    
    close(g_uart_fd);
    return NULL;
}
