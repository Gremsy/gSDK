cmake_minimum_required(VERSION 3.10)
project(gSDK VERSION 3.0.0)

find_package( Threads )

set(MAVLINK_INCLUDES
    "${CMAKE_CURRENT_SOURCE_DIR}/src/mavlink/include/mavlink/v2.0")

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
    ${MAVLINK_SRC}/gimbal_interface.cpp
    ${MAVLINK_SRC}/gimbal_protocol.cpp
    ${MAVLINK_SRC}/gimbal_protocol_v1.cpp
    ${MAVLINK_SRC}/gimbal_protocol_v2.cpp
    ${MAVLINK_SRC}/serial_port.cpp
)

set(GSDK_HEADERS
    mavlink_control.h
    ${MAVLINK_SRC}/gimbal_interface.h
    ${MAVLINK_SRC}/gimbal_protocol.h
    ${MAVLINK_SRC}/gimbal_protocol_v1.h
    ${MAVLINK_SRC}/gimbal_protocol_v2.h
    ${MAVLINK_SRC}/serial_port.h
)

add_library(${PROJECT_NAME} SHARED ${GSDK_APP_SRC})


target_link_libraries(${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT} )

# CMake files
include(CMakePackageConfigHelpers)
write_basic_package_version_file("gsdk-config-version.cmake" VERSION ${PROJECT_VERSION} COMPATIBILITY SameMajorVersion)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/gSDKConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/cmake/gSDKConfig.cmake
  @ONLY)


install(TARGETS ${PROJECT_NAME} EXPORT gsdk-targets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/lib ${CMAKE_INSTALL_INCLUDEDIR}/lib/mavlink
        )
install(FILES
        ${CMAKE_CURRENT_BINARY_DIR}/cmake/gSDKConfig.cmake
        ${PROJECT_BINARY_DIR}/gsdk-config-version.cmake
        DESTINATION lib/cmake/${PROJECT_NAME})
install(EXPORT gsdk-targets
        NAMESPACE gSDK::
        DESTINATION lib/cmake/${PROJECT_NAME}
)
install(FILES ${GSDK_HEADERS}
        DESTINATION include/${PROJECT_NAME})
install(DIRECTORY "${MAVLINK_INCLUDES}/"
        DESTINATION include/mavlink
        FILES_MATCHING PATTERN "*.h")

# add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/examples")
