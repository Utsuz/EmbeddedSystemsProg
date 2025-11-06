add_executable(master_pico
    master_pico.c
)

target_link_libraries(master_pico
    pico_stdlib
    uart_driver
    activation_driver
)

target_include_directories(master_pico PRIVATE
    ${CMAKE_SOURCE_DIR}/../uart_driver
    ${CMAKE_SOURCE_DIR}/../activation_driver
)

pico_enable_stdio_usb(master_pico 1)
pico_enable_stdio_uart(master_pico 0)

pico_add_extra_outputs(master_pico)

