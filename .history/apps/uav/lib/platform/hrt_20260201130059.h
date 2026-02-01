/****************************************************************************
 * apps/uav/lib/platform/hrt.h
 *
 * High-Resolution Timer API cho NuttX
 * Cung cấp thời gian chính xác ở độ phân giải microsecond
 ****************************************************************************/

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Lấy thời gian tuyệt đối tính bằng microseconds
 * @return Số microseconds kể từ khi hệ thống boot
 */
uint64_t hrt_absolute_time(void);

/**
 * @brief Lấy thời gian tuyệt đối tính bằng milliseconds
 * @return Số milliseconds kể từ khi hệ thống boot
 */
uint64_t hrt_absolute_time_ms(void);

/**
 * @brief Sleep trong khoảng thời gian xác định
 * @param us Số microseconds cần sleep
 */
void hrt_usleep(uint32_t us);

#ifdef __cplusplus
}
#endif
