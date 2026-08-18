cmake_minimum_required(VERSION 3.13)

project(computer_simulator C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(computer_simulator
    main.cpp
)

pico_set_program_name(computer_simulator "computer_simulator")
pico_set_program_version(computer_simulator "0.1")

pico_enable_stdio_uart(computer_simulator 0)
pico_enable_stdio_usb(computer_simulator 1)

target_link_libraries(computer_simulator
    pico_stdlib
    hardware_uart
)

pico_add_extra_outputs(computer_simulator)
