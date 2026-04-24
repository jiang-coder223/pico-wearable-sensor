#include <sys/time.h>
#include "hardware/rtc.h"
#include <stdio.h>

void sntp_set_system_time(uint32_t sec)
{
    // ⭐ 1. 更新 libc time()
    struct timeval tv = {
        .tv_sec = sec,
        .tv_usec = 0
    };
    settimeofday(&tv, NULL);

    // ⭐ 2. 更新 RTC（可選，但建議）
    time_t t = sec;
    struct tm *utc = gmtime(&t);

    datetime_t dt = {
        .year  = utc->tm_year + 1900,
        .month = utc->tm_mon + 1,
        .day   = utc->tm_mday,
        .dotw  = utc->tm_wday,
        .hour  = utc->tm_hour,
        .min   = utc->tm_min,
        .sec   = utc->tm_sec
    };

    rtc_set_datetime(&dt);

    printf("[SNTP] synced: %lu\n", sec);
}