// apps/data_logger/include/data_types.h

#ifndef __APPS_DATA_LOGGER_DATA_TYPES_H
#define __APPS_DATA_LOGGER_DATA_TYPES_H

#include <stdint.h>
#include <time.h>

/* Sensor data structure */
struct sensor_data {
    uint32_t timestamp;  // Unix timestamp (seconds)
    uint32_t seq_num;    // Sequence number
    uint8_t sensor_id;   // Sensor identifier
    uint8_t flags;       // Status flags
    float value;         // Sensor value
    float raw_value;     // Raw ADC/sensor value
};

/* Filtered data structure */
struct filtered_data {
    uint32_t timestamp;
    uint8_t sensor_id;
    float value;
    float min;
    float max;
    float avg;
    uint16_t sample_count;
};

/* System status */
struct system_status {
    bool sensors_ok;
    bool storage_ok;
    bool comm_ok;
    uint32_t total_samples;
    uint32_t storage_used_kb;
};

#endif /* __APPS_DATA_LOGGER_DATA_TYPES_H */
