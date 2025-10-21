project(gSDK)

cmake_minimum_required(VERSION 3.5)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(MAVLINK_INCLUDES
    "${CMAKE_CURRENT_SOURCE_DIR}/src/mavlink/c_library_v2")

set(MAVLINK_SRC
    "${CMAKE_CURRENT_SOURCE_DIR}/src")

###############################################################################
#### INCLUDES #################################################################
###############################################################################

include_directories(${CMAKE_CURRENT_SOURCE_DIR}
                    ${CMAKE_CURRENT_SOURCE_DIR}/src
                    ${MAVLINK_INCLUDES}
                    )

###############################################################################
#### DEPENDENCIES #############################################################
###############################################################################

###############################################################################
#### GENERATE OUTPUT ##########################################################
###############################################################################

set(GSDK_APP_SRC
    mavlink_control.cpp
    ${MAVLINK_SRC}/parsers/utils.cpp
    ${MAVLINK_SRC}/bootloader.cpp
    ${MAVLINK_SRC}/gimbal_interface.cpp
    ${MAVLINK_SRC}/gimbal_protocol.cpp
    ${MAVLINK_SRC}/gimbal_protocol_v1.cpp
    ${MAVLINK_SRC}/gimbal_protocol_v2.cpp
    ${MAVLINK_SRC}/serial_port.cpp
)

add_executable(gSDK ${GSDK_APP_SRC})

find_package( Threads )
target_link_libraries(gSDK
                      ${CMAKE_THREAD_LIBS_INIT}
                      )

#install(TARGETS
#        gSDK
#        RUNTIME DESTINATION usr/bin/gSDK)

add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/examples")