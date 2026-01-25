#include "hrt.h"
#include <nuttx/clock.h>
#include <time.h>
#include <unistd.h>

uint64_t hrt_absolute_time(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + ts.tv_nsec / 1000;
}

uint64_t hrt_absolute_time_ms(void)
{
    return hrt_absolute_time() / 1000;
}

void hrt_usleep(uint32_t us)
{
    usleep(us);
}