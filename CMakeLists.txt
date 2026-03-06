cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME H7_Ctrl)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Add CMSIS-DSP
set(CMSISCORE "${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS")
add_subdirectory(D:/PROJECTS/Wheel_Leg_About/Wheel_Leg_VScode/Drivers/CMSIS-DSP-main/Source bin_dsp)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

file(GLOB_RECURSE THREAD
        "IMU_coculate/*.c"
        "others/*.c"
        "UI/*.c"
        "UserMiddlewares/*.c"
        "math/*.c"
        "Power_control/*.c"
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    ${THREAD}
)

# Add include paths
include_directories(
    ${CMAKE_CURRENT_LIST_DIR}/others
    ${CMAKE_CURRENT_LIST_DIR}/IMU_coculate
    ${CMAKE_CURRENT_LIST_DIR}/UI/include
    ${CMAKE_CURRENT_LIST_DIR}/UserMiddlewares/include
    ${CMAKE_CURRENT_LIST_DIR}/math/include
    ${CMAKE_CURRENT_LIST_DIR}/Power_control
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    CMSISDSP
    # Add user defined libraries
)

set(CMAKE_COLOR_DIAGNOSTICS ON)
