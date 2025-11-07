add_library(uart_driver
    uart_driver.c
)

target_link_libraries(uart_driver
    pico_stdlib
)

add_executable(uart_driver_test
    uart_driver_test.c
)

# Link test with UART driver + SDK
target_link_libraries(uart_driver_test
    pico_stdlib
    uart_driver
)

pico_enable_stdio_usb(uart_driver_test 1)
pico_enable_stdio_uart(uart_driver_test 0)

pico_add_extra_outputs(uart_driver_test)

