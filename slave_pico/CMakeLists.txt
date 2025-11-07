# slave_pico/CMakeLists.txt

set(BMP388_DIR ${CMAKE_CURRENT_LIST_DIR}/../bmp388)
set(ACTIVATION_DIR ${CMAKE_CURRENT_LIST_DIR}/../activation_driver)
set(UART_DIR ${CMAKE_CURRENT_LIST_DIR}/../uart_driver)

add_executable(slave_pico
    ${CMAKE_CURRENT_LIST_DIR}/slave_pico.c
    ${BMP388_DIR}/sensor_driver.c
    ${BMP388_DIR}/storage_driver.c
    ${ACTIVATION_DIR}/activation_driver.c
    ${UART_DIR}/uart_driver.c
)

target_include_directories(slave_pico PRIVATE
    ${BMP388_DIR}
    ${ACTIVATION_DIR}
    ${UART_DIR}
)

target_link_libraries(slave_pico
    pico_stdlib
    hardware_i2c
    hardware_flash
    hardware_sync
)

pico_enable_stdio_usb(slave_pico 1)
pico_enable_stdio_uart(slave_pico 0)

pico_add_extra_outputs(slave_pico)
