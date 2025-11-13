add_executable(slave_pico
    slave_pico.c
)

target_link_libraries(slave_pico
    pico_stdlib
    uart_driver
    activation_driver
    bmp388_core
)

target_include_directories(slave_pico PRIVATE
    ${CMAKE_SOURCE_DIR}/../uart_driver
    ${CMAKE_SOURCE_DIR}/../activation_driver
    ${CMAKE_SOURCE_DIR}/../bmp388
)

pico_enable_stdio_usb(slave_pico 1)
pico_enable_stdio_uart(slave_pico 0)

pico_add_extra_outputs(slave_pico)

