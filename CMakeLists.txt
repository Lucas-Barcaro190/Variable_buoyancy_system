# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

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

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(vbs_cpp C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(vbs_cpp vbs_cpp.cpp MS5837.cpp)

pico_set_program_name(vbs_cpp "vbs_cpp")
pico_set_program_version(vbs_cpp "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(vbs_cpp 0)
pico_enable_stdio_usb(vbs_cpp 1)

include(C:/Users/lucas/.pico-sdk/sdk/2.2.0/external/pico_sdk_import.cmake)

include(C:/VBS/FreeRTOS-Kernel/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)

# Add the standard library to the build
target_link_libraries(vbs_cpp
    pico_stdlib
    hardware_adc
    hardware_i2c
    hardware_interp
    hardware_timer
    hardware_watchdog
    hardware_clocks
    pico_i2c_slave
    FreeRTOS-Kernel            # O Kernel base
    FreeRTOS-Kernel-Heap4      # Gerenciamento de memória
)
# Add the standard include files to the build
target_include_directories(vbs_cpp PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

pico_add_extra_outputs(vbs_cpp)