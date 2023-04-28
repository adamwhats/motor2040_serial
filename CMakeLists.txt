cmake_minimum_required(VERSION 3.12)

set(NAME xavbot-motor2040)

include(pimoroni_pico_import.cmake)
include(pico_sdk_import.cmake)

# Gooey boilerplate
project(${NAME} C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()

# Add your source files
add_executable(${NAME}
    ${CMAKE_CURRENT_LIST_DIR}/src/main.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/usb_descriptors.c
)

# Make sure TinyUSB can find tusb_config.h
target_include_directories(${NAME} PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/src)
    
# Include required libraries
# This assumes `pimoroni-pico` is stored alongside your project
include(libraries/motor2040/motor2040)
include(drivers/plasma/plasma)
include(drivers/motor/motor)
include(drivers/motor/motor_cluster)
include(drivers/pwm/pwm)
include(drivers/pwm/pwm_cluster)
include(drivers/encoder/encoder)
include(drivers/button/button)
include(drivers/pid/pid)

# Don't forget to link the libraries you need!
target_link_libraries(${NAME}
    pico_stdlib
    motor2040
    button
    pid    
    tinyusb_device
    tinyusb_board
)

# create map/bin/hex file etc.
pico_add_extra_outputs(${NAME})

# Set up files for the release packages
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${NAME}.uf2
    ${CMAKE_CURRENT_LIST_DIR}/README.md
    DESTINATION .
)

set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY OFF)
set(CPACK_GENERATOR "ZIP" "TGZ")
include(CPack)
