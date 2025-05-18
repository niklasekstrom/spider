cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(spider C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

add_executable(spider spider.c)

pico_enable_stdio_usb(spider 1)
pico_enable_stdio_uart(spider 0)

pico_add_extra_outputs(spider)

target_link_libraries(spider pico_stdlib pico_multicore hardware_spi)
