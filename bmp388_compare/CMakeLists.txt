add_executable(Compare_temp
    Compare_temp.c
    bmp388_driver.c
    bmp388_driver.h
)
target_include_directories(Compare_temp PUBLIC ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(Compare_temp
    pico_stdlib
    hardware_i2c
    hardware_adc     # <-- add this
)

pico_enable_stdio_usb(Compare_temp 1)
pico_enable_stdio_uart(Compare_temp 0)
target_compile_definitions(Compare_temp PRIVATE
    PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS=3000
    PICO_STDIO_USB_LOW_POWER_IDLE_TIMEOUT_MS=0
)
pico_add_extra_outputs(Compare_temp)
