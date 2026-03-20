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

project(variable_buoyancy_system C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(variable_buoyancy_system vbs_cpp.cpp MS5837.cpp)

pico_set_program_name(variable_buoyancy_system "variable_buoyancy_system")
pico_set_program_version(variable_buoyancy_system "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(variable_buoyancy_system 0)
pico_enable_stdio_usb(variable_buoyancy_system 1)

include("C:/Users/Lucas Barcaro/.pico-sdk/sdk/2.2.0/external/pico_sdk_import.cmake")

include("C:/VBS/VBS/FreeRTOS-Kernel/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake")

# Add the standard library to the build
target_link_libraries(variable_buoyancy_system
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
target_include_directories(variable_buoyancy_system PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

pico_add_extra_outputs(variable_buoyancy_system)