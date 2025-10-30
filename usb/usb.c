#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include <string.h>

void usb_init(void) {
  stdio_init_all();
  sleep_ms(2000); // 2 sec delay
  printf("USB initialized.\n");
}

void usb_send(const char *message) {
  if (message) printf("%s", message);
}

void usb_send_raw(const uint8_t *data, size_t length) {
  if (!data || length == 0) return;
  for (size_t i = 0; i < length; i++) {
    putchar(data[i]);
  }
}

bool usb_is_connected(void) {
  return stdio_usb_connected();
}

// ------------------ TEST FUNCTION ------------------

#ifdef USB_STANDALONE_TEST
int main(void) {
    usb_init();

    usb_send("USB Serial Ready!\n");
    usb_send("Time, Temp_BMP388, Temp_OnChip\n");
    
    float bmp_temp = 27.0f;
    float chip_temp = 30.0f;
    int seconds = 0;

    for (int i = 0; i < 20; i++) {
        bmp_temp += 0.1f;
        chip_temp += 0.15f;
        seconds += 1;

        // formatted string output
        char buffer[64];
        snprintf(buffer, sizeof(buffer), "%d,%.2f,%.2f\n", seconds, bmp_temp, chip_temp);
        usb_send(buffer);

        // --- Raw binary send test ---
        // Example: pack time + both temps as bytes
        uint8_t raw_data[12];
        memcpy(raw_data, &seconds, sizeof(int));          // 4 bytes
        memcpy(raw_data + 4, &bmp_temp, sizeof(float));   // 4 bytes
        memcpy(raw_data + 8, &chip_temp, sizeof(float));  // 4 bytes

        usb_send_raw(raw_data, sizeof(raw_data));

        sleep_ms(1000);
    }

    usb_send("Done!\n");

    while (1) {
        tight_loop_contents();
    }
}
#endif

