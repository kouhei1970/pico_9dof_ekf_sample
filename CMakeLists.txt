cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(ekf_quaternion_estimate C CXX ASM)
set(CMAKE_C_STNDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(PICO_EXAMPLES_PATH ${PROJECT_SOURCE_DIR})

pico_sdk_init()
include_directories(/Users/itoukouhei/pico/eigen)

add_executable(ekf_quaternion_estimate
  ekf_quaternion_estimate.cpp
  sensor.cpp
  ekf.cpp
  lsm9ds1_reg.c
)
target_link_libraries(ekf_quaternion_estimate hardware_spi pico_stdlib)

pico_enable_stdio_usb(ekf_quaternion_estimate 1)
pico_enable_stdio_uart(ekf_quaternion_estimate 0)

pico_add_extra_outputs(ekf_quaternion_estimate)

