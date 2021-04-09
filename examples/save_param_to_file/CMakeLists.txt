cmake_minimum_required(VERSION 3.5)

###############################################################################
#### INCLUDES #################################################################
###############################################################################

include_directories(${CMAKE_CURRENT_SOURCE_DIR}
                    ${MAVLINK_INCLUDES}
                    )

###############################################################################
#### DEPENDENCIES #############################################################
###############################################################################

###############################################################################
#### GENERATE OUTPUT ##########################################################
###############################################################################

set(SAVE_PARAM_SRC
    save_param_to_file.cpp
    ${MAVLINK_SRC}/gimbal_interface.cpp
    ${MAVLINK_SRC}/serial_port.cpp
)

add_executable(save_param_to_file ${SAVE_PARAM_SRC})

find_package( Threads )
target_link_libraries(save_param_to_file
                      ${CMAKE_THREAD_LIBS_INIT}
                      )

#install(TARGETS
#        save_param_to_file
#        RUNTIME DESTINATION usr/bin/save_param_to_file)