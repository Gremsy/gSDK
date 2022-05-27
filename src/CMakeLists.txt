project(src)
cmake_minimum_required(VERSION 3.5)

set(MAVLINK_INCLUDES "${CMAKE_CURRENT_SOURCE_DIR}/mavlink/include/mavlink/v2.0")

###############################################################################
#### INCLUDES #################################################################
###############################################################################

## Specify additional locations of header files
include_directories(gimbal_interface
                    hal
                    ${MAVLINK_INCLUDES})

# Use this if more platform is supported
if (CMAKE_SYSTEM_NAME MATCHES Linux)
    include_directories(platform/linux)
endif()

###############################################################################
#### DEPENDENCIES #############################################################
###############################################################################

###############################################################################
#### GENERATE OUTPUT ##########################################################
###############################################################################

## Source code for gSDK
set(GSDK_LIB_SRC
    gimbal_interface/gimbal_interface.cpp
    gimbal_interface/gimbal_protocol.cpp
    gimbal_interface/gimbal_protocol_v1.cpp
    gimbal_interface/gimbal_protocol_v2.cpp
    hal/gsdk_serial_manager.cpp
    hal/gsdk_thread_manager.cpp)

# Use this if more platform is supported
if (CMAKE_SYSTEM_NAME MATCHES Linux)
    set(GSDK_LIB_SRC
        platform/linux/linux_serial_port.cpp
        platform/linux/posix_thread_manager.cpp)
endif()

add_library(${PROJECT_NAME} ${GSDK_LIB_SRC})
find_package( Threads )
