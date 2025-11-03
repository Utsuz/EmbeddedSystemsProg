#include <stdio.h>
#include <string.h>
#include <time.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h" 
#include "FreeRTOS.h" 
#include "task.h"

#include "usb/usb.h"
#include "ntp_driver.h"
#include "HMAC_SHA256/hmac_sha256.h"

// Prints hash
static void print_hex(const uint8_t *b, size_t n) {
    for (size_t i = 0; i < n; ++i)
        printf("%02x", b[i]);
    printf("\n");
}

// FreeRTOS Wi-Fi + NTP
static void wifi_ntp_task(void *pvParameters) {
  stdio_init_all();

  printf("=== Pico Main NTP test (wifi + time sync) ===\n");

  if (cyw43_arch_init_with_country(CYW43_COUNTRY_SINGAPORE)) {
    printf("Wi-Fi init failed!\n");
    vTaskDelete(NULL);
  }

  cyw43_arch_enable_sta_mode();

  printf("Connecting to Wi-Fi SSID: %s\n", WIFI_SSID);
  if (cyw43_arch_wifi_connect_timeout_ms(
    WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
      printf("Failed to connect.\n");
  } else {
    printf("Connected!\n");
    vTaskDelay(pdMS_TO_TICKS(2000));

    time_t now = ntp_get_time();
    if (now) {
      printf("NTP synced local time: %s\n", ctime(&now));
  
      // local clock simulation loop
      while (true) {
        now += 1;
        printf("Local time: %s", ctime(&now));
        vTaskDelay(pdMS_TO_TICKS(1000));
      }
    } else {
      printf("NTP request failed.\n");
    }
  }

  cyw43_arch_deinit();
  vTaskDelete(NULL);
}

int main(void) {
    stdio_init_all();
    sleep_ms(2000); // give USB time to connect

    printf("=== Pico Main HMAC-SHA256 Test ===\n");

    const char *key = "INF2004_KEY";
    const char *msg = "Temperature 28.5C";

    uint8_t hash[32];
    hmac_sha256((const uint8_t*)key, strlen(key),
                (const uint8_t*)msg, strlen(msg),
                hash);

    printf("Key: %s\n", key);
    printf("Msg: %s\n", msg);
    printf("Digest: ");
    print_hex(hash, sizeof(hash));
    printf("\n");

    // Start Wi-Fi + NTP task
    xTaskCreate(wifi_ntp_task, "wifi_ntp_task", 4096, NULL, 1, NULL);
    vTaskStartScheduler();

    // Error, set to standby mode
    while (true) {
      tight_loop_contents();
    }
}
