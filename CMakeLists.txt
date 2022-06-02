project(gSDK DESCRIPTION "Gremsy SDK")
cmake_minimum_required(VERSION 3.5)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/libs)

add_subdirectory(src)

if(${CMAKE_SYSTEM_NAME} MATCHES Linux)
  add_subdirectory(sample/platform/linux)
endif()
