#include "pico/stdlib.h"
#include "temp_sensor.h"
#include <stdio.h>
#include <math.h>

typedef enum { MODE_BASELINE = 0, MODE_EVENT = 1 } log_mode_t;

static const float TLO = 30.0f;    // lower threshold °C
static const float THI = 32.0f;    // upper threshold °C
static const uint32_t BASELINE_MS = 5000;  // baseline interval
static const uint32_t EVENT_MS    = 1000;  // event interval
static const uint32_t RECOVER_MS  = 30000; // must be stable this long to recover

static inline bool in_range(float c) { return (c >= TLO) && (c <= THI); }

int main(void) {
    stdio_init_all();
    sleep_ms(1500);

    if (temp_sensor_init() != 0) {
        // One CSV line indicating init error, then halt
        printf("0,0.00,INIT,E\n");
        while (1) tight_loop_contents();
    }

    absolute_time_t t0 = get_absolute_time();
    absolute_time_t last_sample = t0;
    absolute_time_t first_stable = t0;

    log_mode_t mode = MODE_BASELINE;
    uint32_t interval_ms = BASELINE_MS;

    // CSV header (optional)
    // printf("elapsed_ms,temp_c,mode,flag\n");

    while (true) {
        // Wait until next interval
        sleep_until(delayed_by_ms(last_sample, interval_ms));
        last_sample = get_absolute_time();

        // Read temp with small averaging
        float c = 0.0f;
        char flag = 'N';
        if (temp_sensor_read_celsius(8, &c) != 0 || isnan(c)) {
            flag = 'E';
        }

        // Timestamp (ms since boot start)
        uint32_t elapsed_ms = (uint32_t)to_ms_since_boot(t0);

        // Mode control
        if (flag == 'N') {
            if (!in_range(c)) {
                // Excursion -> EVENT
                mode = MODE_EVENT;
                interval_ms = EVENT_MS;
                first_stable = get_absolute_time(); // reset recovery window
            } else {
                // In range; track stable window for recovery
                if (mode == MODE_EVENT) {
                    // If we have been stable long enough, recover
                    if (absolute_time_diff_us(first_stable, get_absolute_time()) >= (int64_t)RECOVER_MS * 1000) {
                        mode = MODE_BASELINE;
                        interval_ms = BASELINE_MS;
                    }
                } else {
                    // baseline and in range: keep updating stable start
                    first_stable = get_absolute_time();
                }
            }
        } else {
            // On read error we keep current mode but mark the record
        }

        // Print CSV line
        printf("%u,%.2f,%s,%c\n",
               elapsed_ms,
               c,
               (mode == MODE_EVENT ? "EVENT" : "BASELINE"),
               flag);
    }
}
