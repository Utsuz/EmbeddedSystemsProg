# This CMake is meant to live in a subfolder (e.g., temp/) of your project.
# Your top-level CMake should already include pico_sdk_init() and example_auto_set_url.cmake.

add_library(temp_sensor temp_sensor.h temp_sensor.c)

add_executable(temp_example temp_example.c)

target_link_libraries(temp_sensor
    pico_stdlib
    hardware_adc
)

target_link_libraries(temp_example
    pico_stdlib
    hardware_adc
    temp_sensor
)

pico_enable_stdio_usb(temp_example 1)
pico_enable_stdio_uart(temp_example 0)

pico_add_extra_outputs(temp_example)

# Optional; works if your top-level included example_auto_set_url.cmake
example_auto_set_url(temp_example)
