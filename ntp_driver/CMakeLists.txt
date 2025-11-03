# ntp_driver/CMakeLists.txt
add_library(ntp_driver STATIC
    ntp_driver.c
)

# Wi-Fi credentials
target_compile_definitions(ntp_driver PRIVATE
    WIFI_SSID=\"${WIFI_SSID}\"
    WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
    LWIP_SOCKET=1
    LWIP_NETCONN=1
    LWIP_DNS=1
    LWIP_TIMEVAL_PRIVATE=0
    NO_SYS=0
)

# Include paths
target_include_directories(ntp_driver PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/..  # for global config headers
    ${CMAKE_SOURCE_DIR}/main  # for lwipopts.h, FreeRTOSConfig.h
    ${FREERTOS_KERNEL_PATH}/include
    ${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/include
)

# Link libraries for Wi-Fi
target_link_libraries(ntp_driver
    pico_stdlib
    pico_cyw43_arch_lwip_sys_freertos
    FreeRTOS-Kernel-Heap4
)

option(BUILD_NTP_TEST "Build standalone NTP driver test executable" ON)

if (BUILD_NTP_TEST)
    add_executable(ntp_driver_test
        ntp_driver.c    # reuse same file (guard main() with #ifdef BUILD_NTP_TEST)
    )

    target_compile_definitions(ntp_driver_test PRIVATE
        WIFI_SSID=\"${WIFI_SSID}\"
        WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
        LWIP_TIMEVAL_PRIVATE=0
        NO_SYS=0
        LWIP_SOCKET=1
        BUILD_NTP_TEST=1
    )

    target_include_directories(ntp_driver_test PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/..
        ${CMAKE_SOURCE_DIR}/main
    )

    # This one handles Wi-Fi background tasks
    target_link_libraries(ntp_driver_test
        pico_stdlib
        pico_cyw43_arch_lwip_sys_freertos
        FreeRTOS-Kernel-Heap4
    )

    pico_enable_stdio_usb(ntp_driver_test 1)
    pico_enable_stdio_uart(ntp_driver_test 0)
    pico_add_extra_outputs(ntp_driver_test)
endif()
