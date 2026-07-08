# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

cmake_minimum_required(VERSION 3.13)

# Pico SDK — must come before project()
include(pico_sdk_import.cmake)

# micro-ROS pre-built library for Raspberry Pi Pico SDK.
#
# Setup (one-time):
#   cd /home/brian/mecanum_ws/pico/mecanum_base
#   git clone --recursive https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk.git libmicroros
#   cd libmicroros && mkdir build && cd build
#   cmake .. -DMICRO_ROS_DISTRO=humble   # match your ROS2 distro
#   make
#
# The build creates libmicroros.a and generates include/ headers used below.
include(libmicroros/libmicroros.cmake)

project(mecanum_base C CXX ASM)
set(CMAKE_C_STANDARD   11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(mecanum_base mecanum_base.cpp)

target_link_libraries(mecanum_base
    pico_stdlib
    hardware_pwm
    hardware_gpio
    hardware_irq
    hardware_i2c      # MPU9250 IMU on I2C1 (GP26/GP27)
    microros          # micro-ROS client library + Pico USB transport
)

pico_enable_stdio_usb(mecanum_base 1)
pico_enable_stdio_uart(mecanum_base 0)

add_compile_definitions(
    PICO_STDIO_USB_CONNECTION_WITHOUT_DTR=1
    PICO_STDIO_ENABLE_CRLF_SUPPORT=0
    PICO_STDIO_DEFAULT_CRLF=0
)

pico_add_extra_outputs(mecanum_base)

message(STATUS "Build: mecanum_base  →  mecanum_base.uf2")
