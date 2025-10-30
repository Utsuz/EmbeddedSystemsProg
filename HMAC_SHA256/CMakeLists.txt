# Build reusable HMAC driver library
add_library(hmac_sha256
    hmac_sha256.c
)

# Add both the library dir and your config dir
target_include_directories(hmac_sha256 PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/mbedtls_config
)

target_link_libraries(hmac_sha256
    pico_stdlib
    pico_mbedtls
)

# Optional standalone HMAC self-test
option(HMAC_TEST "Build standalone HMAC test" OFF)

if(HMAC_TEST)
    add_executable(hmac_test
        hmac_sha256.c
    )

    # Add config path for the test too
    target_include_directories(hmac_test PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/mbedtls_config
    )

    target_link_libraries(hmac_test
        pico_stdlib
        pico_mbedtls
    )

    target_compile_definitions(hmac_test PRIVATE HMAC_STANDALONE_TEST)
    pico_enable_stdio_usb(hmac_test 1)
    pico_enable_stdio_uart(hmac_test 0)
    pico_add_extra_outputs(hmac_test)
endif()

