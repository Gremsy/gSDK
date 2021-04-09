cmake_minimum_required(VERSION 3.5)

#
# Just add any subdirectories which have CMakeLists.txt files.
#
IF (NOT EXIV2_LIB_BUILD)
  macro(subprojectlist RESULT CURDIR)
      file(GLOB CHILDREN RELATIVE ${CURDIR} ${CURDIR}/*)
      set(DIRLIST "")
      foreach(CHILD ${CHILDREN})
          if(IS_DIRECTORY "${CURDIR}/${CHILD}")
              if (EXISTS "${CURDIR}/${CHILD}/CMakeLists.txt")
                  list(APPEND DIRLIST ${CHILD})
              endif()
          endif()
      endforeach()
      set(${RESULT} ${DIRLIST})
  endmacro()

  subprojectlist(SUBDIRS ${CMAKE_CURRENT_SOURCE_DIR})


  foreach(SUBDIR ${SUBDIRS})
    add_subdirectory(${SUBDIR})
  endforeach()
ENDIF ()
