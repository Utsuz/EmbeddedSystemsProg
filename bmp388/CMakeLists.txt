add_library(bmp388_core STATIC
    sensor_driver.c
    storage_driver.c
    bmp388_driver.h
)

target_include_directories(bmp388_core PUBLIC ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(bmp388_core
    pico_stdlib
    hardware_i2c
    hardware_adc
    hardware_flash
)

add_executable(bmp388
    bmp388.c
)

# Link the executable against the new core library
target_link_libraries(bmp388
    bmp388_core
    pico_stdlib
)

pico_enable_stdio_usb(bmp388 1)
pico_enable_stdio_uart(bmp388 0)
target_compile_definitions(bmp388 PRIVATE
    PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS=3000
    PICO_STDIO_USB_LOW_POWER_IDLE_TIMEOUT_MS=0
)
pico_add_extra_outputs(bmp388)

# optional: tiny log region + faster sampling while testing
target_compile_definitions(bmp388 PRIVATE LOG_REGION_SIZE=8192)
target_compile_definitions(bmp388 PRIVATE SAMPLE_PERIOD_MS=200)
