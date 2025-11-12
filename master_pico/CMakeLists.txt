add_executable(master_pico
    master_pico.c
)

target_link_libraries(master_pico
    pico_stdlib
    usb_driver
    uart_driver
    ntp_driver
    activation_driver
    pico_cyw43_arch_lwip_sys_freertos
    FreeRTOS-Kernel-Heap4
    bmp388_core
)

# Wifi credentials
target_compile_definitions(master_pico PRIVATE
    WIFI_SSID=\"${WIFI_SSID}\"
    WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
    LWIP_SOCKET=1
    LWIP_NETCONN=1
    LWIP_DNS=1
    LWIP_TIMEVAL_PRIVATE=0
    NO_SYS=0
)

target_include_directories(master_pico PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/../uart_driver
    ${CMAKE_SOURCE_DIR}/../activation_driver
    ${FREERTOS_KERNEL_PATH}/include
    ${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/include
)

pico_enable_stdio_usb(master_pico 1)
pico_enable_stdio_uart(master_pico 0)

pico_add_extra_outputs(master_pico)

