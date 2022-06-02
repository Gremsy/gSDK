project(gSDK-linux-sample DESCRIPTION "Gremsy SDK Linux Sample")
cmake_minimum_required(VERSION 3.5)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE
      Debug
      CACHE STRING "Choose the type of build Debug/Release/Coverage:" FORCE)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread -g -O0 -Wall")

if(NOT MAVLINK_INCLUDES)
  set(MAVLINK_INCLUDES ${gSDK_core_SOURCE_DIR}/mavlink/include/mavlink/v2.0)
endif()

# Specify additional locations of header files
include_directories(
  ${gSDK_core_SOURCE_DIR}/gimbal_interface ${gSDK_core_SOURCE_DIR}/hal
  ${MAVLINK_INCLUDES} ${gSDK_core_SOURCE_DIR}/platform/linux)

# Source code for gSDK Sample
set(GSDK_SAMPLE_SRC mavlink_control.cpp)

add_executable(${PROJECT_NAME} ${GSDK_SAMPLE_SRC})
target_link_libraries(${PROJECT_NAME} PUBLIC gSDK_core)
