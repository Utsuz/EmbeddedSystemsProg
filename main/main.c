#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "usb/usb.h"
#include "HMAC_SHA256/hmac_sha256.h"

// Prints hash
static void print_hex(const uint8_t *b, size_t n) {
    for (size_t i = 0; i < n; ++i)
        printf("%02x", b[i]);
    printf("\n");
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

    printf("Done!\n");

    while (true) {
        tight_loop_contents();
    }
}
