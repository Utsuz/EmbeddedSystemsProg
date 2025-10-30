#include "hmac_sha256.h"
#include <string.h>
#include "mbedtls/sha256.h"

static void sha256(const uint8_t *data, size_t len, uint8_t out[32]) {
    mbedtls_sha256_context c;
    mbedtls_sha256_init(&c);
    mbedtls_sha256_starts(&c, 0);
    mbedtls_sha256_update(&c, data, len);
    mbedtls_sha256_finish(&c, out);
    mbedtls_sha256_free(&c);
}

void hmac_sha256(const uint8_t *key, size_t key_len,
                 const uint8_t *msg, size_t msg_len,
                 uint8_t out[32]) {
    uint8_t k0[64] = {0};
    uint8_t ipad[64], opad[64];

    if (key_len > 64) {
        sha256(key, key_len, k0);
    } else {
        memcpy(k0, key, key_len);
    }
    for (int i = 0; i < 64; ++i) {
        ipad[i] = k0[i] ^ 0x36;
        opad[i] = k0[i] ^ 0x5c;
    }

    uint8_t inner[64 + msg_len]; 
    memcpy(inner, ipad, 64);
    memcpy(inner + 64, msg, msg_len);

    uint8_t inner_hash[32];
    sha256(inner, sizeof(inner), inner_hash);

    uint8_t outer[64 + 32];
    memcpy(outer, opad, 64);
    memcpy(outer + 64, inner_hash, 32);
    sha256(outer, sizeof(outer), out);
}

#ifdef HMAC_STANDALONE_TEST
#include <stdio.h>
#include "pico/stdlib.h"

static void print_hex(const uint8_t *b, size_t n) {
    for (size_t i = 0; i < n; ++i) printf("%02x", b[i]);
    printf("\n");
}

int main(void) {
    stdio_init_all();
    sleep_ms(2000);

    const char *key = "INF2004_KEY";
    const char *msg = "HMAC test message from Pico w";

    uint8_t mac[32];
    hmac_sha256((const uint8_t*)key, strlen(key),
                (const uint8_t*)msg, strlen(msg),
                mac);

    printf("=== HMAC-SHA256 Test ===\nKey: %s\nMsg: %s\nHMAC: ", key, msg);
    print_hex(mac, sizeof mac);

    while (1) tight_loop_contents();
}
#endif
