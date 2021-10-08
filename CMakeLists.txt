cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(ekf_quatnion_estimate C CXX ASM)
set(CMAKE_C_STNDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(PICO_EXAMPLES_PATH ${PROJECT_SOURCE_DIR})

pico_sdk_init()

add_executable(ekf_quatnion_estimate
  ekf_quatnion_estimate.cpp
  lsm9ds1_reg.c
)
target_link_libraries(ekf_quatnion_estimate hardware_spi pico_stdlib)

pico_enable_stdio_usb(ekf_quatnion_estimate 1)
pico_enable_stdio_uart(ekf_quatnion_estimate 0)

pico_add_extra_outputs(ekf_quatnion_estimate)

