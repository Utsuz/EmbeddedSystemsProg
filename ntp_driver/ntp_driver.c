#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "ntp_driver.h"
#include "lwip/opt.h"
#include "lwip/netdb.h"
#include "lwip/api.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netif.h"
#include "lwip/dns.h"
#include "lwip/inet.h"
#include "lwip/err.h"
#include <string.h>
#include <stdio.h>
#include <time.h>

#define NTP_SERVER       "pool.ntp.org"
#define NTP_PORT         123
#define NTP_MSG_LEN      48
#define NTP_UNIX_OFFSET  2208988800UL  // Seconds between 1900 and 1970
#define TIMEZONE_OFFSET  (8 * 60 * 60) // Singapore UTC+8

// ----------------------------------------------------
// Initialize Wi-Fi (STA mode) for NTP usage
// ----------------------------------------------------
void ntp_init(void) {
    if (cyw43_arch_init()) {
        printf("Wi-Fi init failed!\n");
        return;
    }

    cyw43_arch_enable_sta_mode();
    printf("Connecting to Wi-Fi SSID: %s...\n", WIFI_SSID);

    int result = cyw43_arch_wifi_connect_timeout_ms(
        WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000);

    if (result) {
        printf("Wi-Fi connection failed (code %d)\n", result);
    } else {
        printf("Wi-Fi connected successfully!\n");
    }
}

// ----------------------------------------------------
// Perform a blocking NTP request and return Unix time
// ----------------------------------------------------
time_t ntp_get_time(void) {
    int sock;
    struct sockaddr_in server_addr;
    uint8_t msg[NTP_MSG_LEN] = {0};
    uint8_t recv_buf[NTP_MSG_LEN];

    // NTP request flags (LI=0, VN=3, Mode=3)
    msg[0] = 0x1B;

    // Resolve DNS for NTP server
    ip_addr_t ntp_ip;
    err_t dns_result = netconn_gethostbyname(NTP_SERVER, &ntp_ip);
    if (dns_result != ERR_OK) {
        printf("DNS resolution failed for %s (err %d)\n", NTP_SERVER, dns_result);
        return 0;
    }

    server_addr.sin_family = AF_INET;
    server_addr.sin_port = PP_HTONS(NTP_PORT);
    server_addr.sin_addr.s_addr = ntp_ip.addr;

    // Open UDP socket
    sock = lwip_socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        printf("Socket create failed\n");
        return 0;
    }

    // Send request
    if (lwip_sendto(sock, msg, NTP_MSG_LEN, 0,
                    (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        printf("Send failed\n");
        lwip_close(sock);
        return 0;
    }

    // Receive response
    struct sockaddr_in from;
    socklen_t from_len = sizeof(from);
    int recv_len = lwip_recvfrom(sock, recv_buf, sizeof(recv_buf), 0,
                                 (struct sockaddr*)&from, &from_len);
    lwip_close(sock);

    if (recv_len < NTP_MSG_LEN) {
        printf("Invalid NTP reply\n");
        return 0;
    }

    // Extract timestamp (bytes 40-43)
    uint32_t seconds_since_1900 =
        (recv_buf[40] << 24) | (recv_buf[41] << 16) | (recv_buf[42] << 8) | recv_buf[43];
    time_t unix_time = (seconds_since_1900 - NTP_UNIX_OFFSET) + TIMEZONE_OFFSET;

    return unix_time;
}

// ----------------------------------------------------
// Standalone test main() to verify functionality
// ----------------------------------------------------

#ifdef BUILD_NTP_TEST
#include "FreeRTOS.h"
#include "task.h"
#include "pico/cyw43_arch.h"
#include <stdio.h>

void wifi_task(void *pvParameters) {
    stdio_init_all();

    if (cyw43_arch_init_with_country(CYW43_COUNTRY_SINGAPORE)) {
        printf("WiFi init failed\n");
        vTaskDelete(NULL);
    }

    cyw43_arch_enable_sta_mode();

    printf("Connecting to WiFi SSID: %s\n", WIFI_SSID);
    if (cyw43_arch_wifi_connect_timeout_ms(
            WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("Failed to connect.\n");
    } else {
        printf("Connected!\n");
        vTaskDelay(pdMS_TO_TICKS(2000));

        time_t now = ntp_get_time();
        if (now)
            printf("NTP time: %s\n", ctime(&now));
        else
            printf("NTP request failed.\n");
    }

    cyw43_arch_deinit();
    vTaskDelete(NULL);
}

int main(void) {
    // Create the Wi-Fi / NTP task
    xTaskCreate(wifi_task, "wifi_task", 4096, NULL, 1, NULL);

    // Start the FreeRTOS scheduler (this lets the Wi-Fi driver run)
    vTaskStartScheduler();

    // We should never reach here
    while (true) {
        tight_loop_contents();
    }
}
#endif




