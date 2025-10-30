#ifndef MBEDTLS_CONFIG_H
#define MBEDTLS_CONFIG_H

/* Enable basic crypto features for SHA256 / HMAC */
#define MBEDTLS_SHA256_C
#define MBEDTLS_MD_C
#define MBEDTLS_PLATFORM_C
#define MBEDTLS_ERROR_C

/* No filesystem, threading, or network needed on Pico */
#undef MBEDTLS_FS_IO
#undef MBEDTLS_NET_C
#undef MBEDTLS_THREADING_C

#endif /* MBEDTLS_CONFIG_H */

