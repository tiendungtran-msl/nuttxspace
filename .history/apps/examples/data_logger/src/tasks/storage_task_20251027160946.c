// apps/data_logger/src/tasks/storage_task.c

#include <nuttx/config.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>

#include "config.h"
#include "data_types.h"
#include "ipc.h"
#include "tasks.h"

#define LOG_FILE_PATH "/mnt/sdcard/datalog.csv"
#define MAX_FILE_SIZE (10 * 1024 * 1024)  // 10MB

static int g_log_fd = -1;
static uint32_t g_bytes_written = 0;

static int storage_open_file(void)
{
    /* Create directory if not exists */
    mkdir("/mnt/sdcard", 0777);
    
    /* Open log file (append mode) */
    g_log_fd = open(LOG_FILE_PATH, O_WRONLY | O_CREAT | O_APPEND, 0644);
    if (g_log_fd < 0) {
        fprintf(stderr, "Storage: Failed to open log file: %d\n", errno);
        return -errno;
    }
    
    /* Get current file size */
    struct stat st;
    if (fstat(g_log_fd, &st) == 0) {
        g_bytes_written = st.st_size;
    }
    
    printf("Storage: Log file opened (%u bytes)\n", g_bytes_written);
    return OK;
}

static void storage_rotate_file(void)
{
    char backup_path[64];
    
    close(g_log_fd);
    
    /* Rename current file */
    snprintf(backup_path, sizeof(backup_path), 
             "/mnt/sdcard/datalog_%u.csv", (unsigned int)time(NULL));
    rename(LOG_FILE_PATH, backup_path);
    
    printf("Storage: File rotated to %s\n", backup_path);
    
    /* Open new file */
    g_bytes_written = 0;
    storage_open_file();
}

void *storage_task(void *arg)
{
    struct filtered_data data;
    char line[128];
    int ret;
    
    printf("Storage task started (tid=%lu)\n",
           (unsigned long)pthread_self());
    
    watchdog_register(pthread_self(), "storage_task");
    
    /* Open log file */
    ret = storage_open_file();
    if (ret < 0) {
        pthread_mutex_lock(&g_status_mutex);
        g_system_status.storage_ok = false;
        pthread_mutex_unlock(&g_status_mutex);
        return NULL;
    }
    
    pthread_mutex_lock(&g_status_mutex);
    g_system_status.storage_ok = true;
    pthread_mutex_unlock(&g_status_mutex);
    
    /* Write CSV header if new file */
    if (g_bytes_written == 0) {
        const char *header = "timestamp,sensor_id,value,min,max,avg\n";
        write(g_log_fd, header, strlen(header));
        g_bytes_written += strlen(header);
    }
    
    while (1) {
        /* Receive filtered data */
        ssize_t nbytes = mq_receive(g_filter_to_storage_mq,
                                     (char *)&data,
                                     sizeof(data), NULL);
        if (nbytes != sizeof(data)) continue;
        
        /* Format data line */
        int len = snprintf(line, sizeof(line),
                          "%u,%u,%.2f,%.2f,%.2f,%.2f\n",
                          data.timestamp,
                          data.sensor_id,
                          data.value,
                          data.min,
                          data.max,
                          data.avg);
        
        /* Write to file */
        ssize_t written = write(g_log_fd, line, len);
        if (written > 0) {
            g_bytes_written += written;
            
            /* Sync to disk periodically */
            static int sync_counter = 0;
            if (++sync_counter >= 10) {
                fsync(g_log_fd);
                sync_counter = 0;
            }
            
            /* Update storage status */
            pthread_mutex_lock(&g_status_mutex);
            g_system_status.storage_used_kb = g_bytes_written / 1024;
            pthread_mutex_unlock(&g_status_mutex);
            
            /* Rotate file if too large */
            if (g_bytes_written > MAX_FILE_SIZE) {
                storage_rotate_file();
            }
        }
        
        watchdog_kick(pthread_self());
    }
    
    close(g_log_fd);
    return NULL;
}
