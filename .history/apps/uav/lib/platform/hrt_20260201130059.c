/****************************************************************************
 * apps/uav/lib/platform/hrt.c
 *
 * Triển khai High-Resolution Timer sử dụng NuttX clock API
 ****************************************************************************/

#include "hrt.h"
#include <nuttx/clock.h>
#include <time.h>
#include <unistd.h>

uint64_t hrt_absolute_time(void)
{
    struct timespec ts;
    clock_systime_timespec(&ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}

uint64_t hrt_absolute_time_ms(void)
{
    return hrt_absolute_time() / 1000ULL;
}

void hrt_usleep(uint32_t us)
{
    usleep(us);
}
