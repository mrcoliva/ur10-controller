cmake_minimum_required(VERSION 2.8.3)
project(ow_msgs)

###################
## Configuration ##
###################

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

# Set the rpath (to build deb package)
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# ROS dependecies
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  AngularState.msg
  CartesianState.msg
  JointState.msg
  LinearState.msg
  Vector.msg
)

# Generate services in the 'srv' folder
# add_service_files(
#   FILES
# )

# Generate actions in the 'action' folder
#add_action_files(
#  FILES
#  Action1.action
#  Action2.action
#)

# Generate added messages and services with any dependencies listed here
# see package.xml and find_package dependencies
generate_messages(
  DEPENDENCIES
  std_msgs  
  geometry_msgs  
)

# Package configuration (this information is passed to projects/packages that
# depend on this package)
# Most of the cases the msgs packages won't generate include or libraries 
catkin_package(
    #   INCLUDE_DIRS include
    #    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
        roscpp
        rospy
        std_msgs
        geometry_msgs
        message_runtime
#    DEPENDS ${DEPENDS_VAR}
)

###########
## Build ##
###########

# Location of header files (First package headers always!)
include_directories(
    #    include
    ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

foreach(dir msg)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#)

#############
## Testing ##
#############

#set(TEST_TARGETS
#    #conversions
#    vector3
#    #cartesian_position
#    #homogeneous_transformation
#    #cartesian_velocity
#    #cartesian_state
#)

#foreach(target ${TEST_TARGETS})
#    catkin_add_gtest(${target} src/tests/test_${target}.cpp)
#    add_dependencies(${target}
#        ${${PROJECT_NAME}_EXPORTED_TARGETS}
#        ${catkin_EXPORTED_TARGETS})
#
#    set_target_properties(${target} PROPERTIES CXX_STANDARD 11)
#
#    target_link_libraries(${target}
#        ${PROJECT_NAME}
#        ${catkin_LIBRARIES}
#    )
#endforeach()
