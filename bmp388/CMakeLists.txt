# bmp388/CMakeLists.txt

add_library(bmp388_driver
    bmp388_driver.c
    bmp388_driver.h
    ${CMAKE_SOURCE_DIR}/lib/bmp3/bmp3.c
)

target_include_directories(bmp388_driver PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_SOURCE_DIR}/lib/bmp3
)

target_link_libraries(bmp388_driver pico_stdlib hardware_i2c hardware_adc)

# Example application
add_executable(bmp388_example bmp388_example.c)

target_link_libraries(bmp388_example
    pico_stdlib
    hardware_i2c
    bmp388_driver
)

pico_enable_stdio_usb(bmp388_example 1)
pico_enable_stdio_uart(bmp388_example 0)

target_compile_definitions(bmp388_example PRIVATE
    PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS=3000
    PICO_STDIO_USB_LOW_POWER_IDLE_TIMEOUT_MS=0
)

pico_add_extra_outputs(bmp388_example)

# Optional: auto-set URL
example_auto_set_url(bmp388_example)

# Scanner tool (optional)
add_executable(i2c_scan_example i2c_scan_example.c)
target_link_libraries(i2c_scan_example pico_stdlib hardware_i2c)
pico_enable_stdio_usb(i2c_scan_example 1)
pico_enable_stdio_uart(i2c_scan_example 0)
pico_add_extra_outputs(i2c_scan_example)
