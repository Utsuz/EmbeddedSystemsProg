add_library(activation_driver
    activation_driver.c
)

target_link_libraries(activation_driver
    pico_stdlib
    uart_driver
)

target_include_directories(activation_driver PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/uart_driver
)

