# Standalone USB test
add_executable(usb_test
    usb.c
)

target_link_libraries(usb_test
    pico_stdlib
)

target_compile_definitions(usb_test PRIVATE USB_STANDALONE_TEST)

pico_enable_stdio_usb(usb_test 1)
pico_enable_stdio_uart(usb_test 0)
pico_add_extra_outputs(usb_test)

# Reusable USB driver library
add_library(usb_driver
    usb.c
)

target_include_directories(usb_driver PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(usb_driver
    pico_stdlib
    pico_stdio_usb
)
