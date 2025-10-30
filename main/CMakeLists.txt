add_executable(main
        main.c
        )

# pull in common dependencies
target_link_libraries(main 
    pico_stdlib
    pico_mbedtls
    hmac_sha256
    usb_driver
    )
    
target_include_directories(main PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/..
)

pico_enable_stdio_usb(main 1)
pico_enable_stdio_uart(main 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(main)
