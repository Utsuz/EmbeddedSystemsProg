#ifndef NTP_DRIVER_H
#define NTP_DRIVER_H

#include <time.h>

// Initialize Wi-Fi and prepare for NTP communication
void ntp_init(void);

// Fetch current time from NTP server (blocking)
time_t ntp_get_time(void);

#endif // NTP_DRIVER_H

