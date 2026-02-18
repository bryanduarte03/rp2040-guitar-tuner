cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(guitar_tuner_lcd C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(guitar_tuner_lcd
    main.c
)

target_link_libraries(guitar_tuner_lcd
    pico_stdlib
    hardware_adc
    hardware_dma
    hardware_i2c
)

pico_enable_stdio_usb(guitar_tuner_lcd 1)
pico_enable_stdio_uart(guitar_tuner_lcd 0)

pico_add_extra_outputs(guitar_tuner_lcd)
