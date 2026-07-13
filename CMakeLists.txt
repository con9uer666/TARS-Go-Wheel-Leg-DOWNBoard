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


# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C CXX ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Add CMSIS-DSP
set(CMSISCORE "${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS")
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS-DSP-main/Source bin_dsp)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

file(GLOB_RECURSE THREAD CONFIGURE_DEPENDS
        "IMU_coculate/*.c"
        "User_code/*.c"
        "User_code/Device_drives/src/*.c"
        "User_code/Motion_Control/App/*.c"
        "User_code/Motion_Control/App/src/*.c"
        "User_code/Motion_Control/Ctrl/src/*.c"
        "User_code/Motion_Control/Power_control/*.c"
        "User_code/CommonLibs/src/*.c"
        "User_code/UI/source/*.c"
        "User_code/UI/source/*.cpp"
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    ${THREAD}
)

# Add include paths
include_directories(
    ${CMAKE_CURRENT_LIST_DIR}/IMU_coculate
    ${CMAKE_CURRENT_LIST_DIR}/User_code
    ${CMAKE_CURRENT_LIST_DIR}/User_code/Device_drives/inc
    ${CMAKE_CURRENT_LIST_DIR}/User_code/Motion_Control/App
    ${CMAKE_CURRENT_LIST_DIR}/User_code/Motion_Control/App/inc
    ${CMAKE_CURRENT_LIST_DIR}/User_code/Motion_Control/Ctrl/inc
    ${CMAKE_CURRENT_LIST_DIR}/User_code/Motion_Control/Power_control
    ${CMAKE_CURRENT_LIST_DIR}/User_code/CommonLibs/inc
    ${CMAKE_CURRENT_LIST_DIR}/User_code/UI/include
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
