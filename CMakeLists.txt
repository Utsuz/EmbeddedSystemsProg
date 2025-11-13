cmake_minimum_required(VERSION 3.12)

set(PICO_CYW43_ARCH_DEFAULT pico_cyw43_arch_lwip_threadsafe_background)

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)
include(pico_extras_import_optional.cmake)

set(FREERTOS_KERNEL_PATH "${CMAKE_CURRENT_SOURCE_DIR}/FreeRTOS-Kernel")
include(${CMAKE_CURRENT_SOURCE_DIR}/FreeRTOS_Kernel_import.cmake)

project(TempSensor C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(PICO_BOARD pico_w)

# if (PICO_SDK_VERSION_STRING VERSION_LESS "2.2.0")
#     message(FATAL_ERROR "Raspberry Pi Pico SDK version 2.2.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
# endif()

# If you want debug output from USB (pass -DPICO_STDIO_USB=1) this ensures you don't lose any debug output while USB is set up
if (NOT DEFINED PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS)
    set(PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS 3000)
endif()

# Initialize the SDK
pico_sdk_init()

# Wifi (2.4 GHz only)
if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/secrets.cmake")
    include("${CMAKE_CURRENT_SOURCE_DIR}/secrets.cmake")
    message(STATUS "Loaded Wi-Fi credentials from secrets.cmake")
else()
    message(WARNING "secrets.cmake not found. Using placeholder Wi-Fi credentials.")
    set(WIFI_SSID "placeholder_ssid")
    set(WIFI_PASSWORD "placeholder_password")
endif()

add_compile_options(-Wall
        -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
        -Wno-unused-function # we have some for the docs that aren't called
        )
if (CMAKE_C_COMPILER_ID STREQUAL "GNU")
    add_compile_options(-Wno-maybe-uninitialized)
endif()

# Hardware-specific examples in subdirectories:
add_subdirectory(bmp388)
add_subdirectory(ntp_driver)
add_subdirectory(activation_driver)
add_subdirectory(master_pico)
add_subdirectory(slave_pico)
add_subdirectory(usb)
add_subdirectory(uart_driver)
add_subdirectory(HMAC_SHA256)

