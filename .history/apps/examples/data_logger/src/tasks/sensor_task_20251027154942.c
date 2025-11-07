// apps/data_logger/src/tasks/sensor_task.c

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <errno.h>

#include "config.h"
#include "data_types.h"
#include "ipc.h"
#include "tasks.h"

/* Simulated sensor reading (replace with real driver) */
static float read_sensor_i2c(void)
{
    /* TODO: Replace with real I2C sensor read */
    return 20.0f + ((float)rand() / RAND_MAX) * 10.0f;  // 20-30°C
}

static float read_sensor_spi(void)
{
    /* TODO: Replace with real SPI sensor read */
    return 50.0f + ((float)rand() / RAND_MAX) * 30.0f;  // 50-80%
}

static float read_sensor_adc(void)
{
    /* TODO: Replace with real ADC read */
    return 3.3f * ((float)rand() / RAND_MAX);  // 0-3.3V
}

void *sensor_task(void *arg)
{
    struct sensor_data data;
    uint32_t seq_num = 0;
    int ret;
    
    printf("Sensor task started (tid=%lu)\n", 
           (unsigned long)pthread_self());
    
    /* Register with watchdog */
    watchdog_register(pthread_self(), "sensor_task");
    
    /* Mark sensors as OK */
    pthread_mutex_lock(&g_status_mutex);
    g_system_status.sensors_ok = true;
    pthread_mutex_unlock(&g_status_mutex);
    
    while (1) {
        /* Read sensor 1 (I2C - Temperature) */
        data.timestamp = time(NULL);
        data.seq_num = seq_num++;
        data.sensor_id = 1;
        data.flags = 0;
        data.raw_value = read_sensor_i2c();
        data.value = data.raw_value;  // No conversion needed
        
        ret = mq_send(g_sensor_to_filter_mq, (char *)&data, 
                      sizeof(data), 0);
        if (ret < 0) {
            fprintf(stderr, "Sensor: Failed to send data (sensor 1)\n");
        }
        
        /* Read sensor 2 (SPI - Humidity) */
        data.timestamp = time(NULL);
        data.seq_num = seq_num++;
        data.sensor_id = 2;
        data.raw_value = read_sensor_spi();
        data.value = data.raw_value;
        
        ret = mq_send(g_sensor_to_filter_mq, (char *)&data,
                      sizeof(data), 0);
        if (ret < 0) {
            fprintf(stderr, "Sensor: Failed to send data (sensor 2)\n");
        }
        
        /* Read sensor 3 (ADC - Voltage) */
        data.timestamp = time(NULL);
        data.seq_num = seq_num++;
        data.sensor_id = 3;
        data.raw_value = read_sensor_adc();
        data.value = data.raw_value;
        
        ret = mq_send(g_sensor_to_filter_mq, (char *)&data,
                      sizeof(data), 0);
        if (ret < 0) {
            fprintf(stderr, "Sensor: Failed to send data (sensor 3)\n");
        }
        
        /* Update total samples */
        pthread_mutex_lock(&g_status_mutex);
        g_system_status.total_samples += 3;
        pthread_mutex_unlock(&g_status_mutex);
        
        /* Kick watchdog */
        watchdog_kick(pthread_self());
        
        /* Sleep until next sampling */
        usleep(CONFIG_DATA_LOGGER_SENSOR_INTERVAL * 1000);
    }
    
    return NULL;
}
