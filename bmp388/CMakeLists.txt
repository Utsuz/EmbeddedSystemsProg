add_executable(bmp388_example

    bmp388_example.c
    bmp388_driver.c
    bmp388_driver.h
)
target_include_directories(bmp388_example PUBLIC ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(bmp388_example
    pico_stdlib
    hardware_i2c
    hardware_adc
)

pico_enable_stdio_usb(bmp388_example 1)
pico_enable_stdio_uart(bmp388_example 0)
target_compile_definitions(bmp388_example PRIVATE
    PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS=3000
    PICO_STDIO_USB_LOW_POWER_IDLE_TIMEOUT_MS=0
)
pico_add_extra_outputs(bmp388_example)

target_compile_definitions(bmp388_example PRIVATE LOG_REGION_SIZE=8192)  # 8 KB for testing
target_compile_definitions(bmp388_example PRIVATE SAMPLE_PERIOD_MS=200)  # faster sampling

