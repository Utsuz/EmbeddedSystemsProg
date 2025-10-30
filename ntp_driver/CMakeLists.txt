add_executable(ntp_driver
        ntp_driver.c
        )
        
target_compile_definitions(ntp_driver PRIVATE
        WIFI_SSID=\"${WIFI_SSID}\"
        WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
        NO_SYS=0            # don't want NO_SYS (generally this would be in your lwipopts.h)
        LWIP_SOCKET=1       # we need the socket API (generally this would be in your lwipopts.h)
        )
        
target_include_directories(ntp_driver PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/.. # for our common FreeRTOSConfig and common lwipopts
        ${FREERTOS_KERNEL_PATH}/include
        ${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/include
        )
        
target_link_libraries(ntp_driver
	pico_stdlib
        pico_cyw43_arch_lwip_sys_freertos
        FreeRTOS-Kernel-Heap4 # FreeRTOS kernel and dynamic heap
        )
        
# --- STDIO configuration ---
pico_enable_stdio_usb(ntp_driver 1)
pico_enable_stdio_uart(ntp_driver 0)

pico_add_extra_outputs(ntp_driver)
