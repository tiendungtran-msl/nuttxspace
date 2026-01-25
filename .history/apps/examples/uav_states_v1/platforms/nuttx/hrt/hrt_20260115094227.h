#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Get absolute time in microseconds
 */
uint64_t hrt_absolute_time(void);

/**
 * Get absolute time in milliseconds
 */
uint64_t hrt_absolute_time_ms(void);

/**
 * Sleep for microseconds
 */
void hrt_usleep(uint32_t us);

#ifdef __cplusplus
}
#endif