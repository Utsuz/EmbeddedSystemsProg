add_executable(main
        main.c
        )

# pull in common dependencies
target_link_libraries(main 
    pico_stdlib
    pico_mbedtls
    hmac_sha256
    usb_driver
    ntp_driver
    pico_cyw43_arch_lwip_sys_freertos
    FreeRTOS-Kernel-Heap4
    )
 
# Wifi credentials
target_compile_definitions(main PRIVATE
    WIFI_SSID=\"${WIFI_SSID}\"
    WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
    LWIP_SOCKET=1
    LWIP_NETCONN=1
    LWIP_DNS=1
    LWIP_TIMEVAL_PRIVATE=0
    NO_SYS=0
)

target_include_directories(main PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${FREERTOS_KERNEL_PATH}/include
    ${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/include
)

pico_enable_stdio_usb(main 1)
pico_enable_stdio_uart(main 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(main)
