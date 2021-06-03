cmake_minimum_required(VERSION 2.8.3)
project(ow_core)

###################
## Configuration ##
###################

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# ROS dependecies
find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf
    tf_conversions
    ow_msgs
)

# System dependencies
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})


## add Qt Modules
#find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
##set(QT_USE_QTNETWORK TRUE)
#
#include(${QT_USE_FILE})
## USE (enforces Q_SIGNAL etc. macros)
##add_definitions(-DQT_NO_KEYWORDS)


#message(STATUS "EIGEN3_INCLUDE_DIR = " ${EIGEN3_INCLUDE_DIR})

# Package configuration (this information is passed to projects/packages that
# depend on this package)
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
        roscpp
        tf
        tf_conversions
        ow_msgs
        
#    DEPENDS ${DEPENDS_VAR}
)

###########
## Build ##
###########

# Location of header files (First package headers always!)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

# needed for Qt generated files
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
)

### Binaries ###
## C++ Library
set(SOURCES
     src/conversions.cpp
     src/math/reference_frames.cpp
)

add_library(${PROJECT_NAME} ${SOURCES})

# Library dependencies. Always use:
# ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
add_dependencies(${PROJECT_NAME}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS})

# Special CXXFlags for the libraries
# set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 11

# linking libraries (from ros packages and system libraries)
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
)

## C++ Executables

set(APP_TARGETS
)

foreach(target ${APP_TARGETS})
        add_executable(${target} src/applications/main_${target}.cpp)
        add_dependencies(${target}
            ${${PROJECT_NAME}_EXPORTED_TARGETS}
            ${catkin_EXPORTED_TARGETS})

        # Special CXXFlags for the binaries
        # set_target_properties(${target} PROPERTIES CXX_STANDARD 11)

        # Generate a short name and define prefix for the target
        # set_target_properties(
        # ${target} PROPERTIES OUTPUT_NAME node PREFIX "")

        target_link_libraries(${target}
            ${PROJECT_NAME}
            ${catkin_LIBRARIES}
        )
endforeach()

#############
## Install ##
#############

# don't install private headers
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    PATTERN "*_p.h" EXCLUDE
    PATTERN "*.h"
)

install(TARGETS ${PROJECT_NAME} ${APP_TARGETS}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )



##Additional dirs and files
#install(DIRECTORY launch/
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
#)

# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )


#############
## Testing ##
#############

set(TEST_TARGETS
)
if(CATKIN_ENABLE_TESTING)
foreach(target ${TEST_TARGETS})
  catkin_add_gtest(${target} src/tests/test_${target}.cpp)
  add_dependencies(${target}
      ${${PROJECT_NAME}_EXPORTED_TARGETS}
      ${catkin_EXPORTED_TARGETS})

  set_target_properties(${target} PROPERTIES CXX_STANDARD 11)

  target_link_libraries(${target}
      ${PROJECT_NAME}
      ${catkin_LIBRARIES}
  )
endforeach()
endif()

