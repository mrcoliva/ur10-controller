# Install script for directory: /home/marco/ros/workspaces/final_project/src/tum_ics_ur10_controller_tutorial

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/marco/ros/workspaces/final_project/src/build/tum_ics_ur10_controller_tutorial/catkin_generated/installspace/tum_ics_ur10_controller_tutorial.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tum_ics_ur10_controller_tutorial/cmake" TYPE FILE FILES
    "/home/marco/ros/workspaces/final_project/src/build/tum_ics_ur10_controller_tutorial/catkin_generated/installspace/tum_ics_ur10_controller_tutorialConfig.cmake"
    "/home/marco/ros/workspaces/final_project/src/build/tum_ics_ur10_controller_tutorial/catkin_generated/installspace/tum_ics_ur10_controller_tutorialConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tum_ics_ur10_controller_tutorial" TYPE FILE FILES "/home/marco/ros/workspaces/final_project/src/tum_ics_ur10_controller_tutorial/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/tum_ics_ur10_controller_tutorial" TYPE DIRECTORY FILES "/home/marco/ros/workspaces/final_project/src/tum_ics_ur10_controller_tutorial/include/tum_ics_ur10_controller_tutorial/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libmy_robot_control_lli.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libmy_robot_control_lli.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libmy_robot_control_lli.so"
         RPATH "/opt/ros/melodic/lib:/usr/lib/tumtools")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/marco/ros/workspaces/final_project/src/build/devel/lib/libmy_robot_control_lli.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libmy_robot_control_lli.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libmy_robot_control_lli.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libmy_robot_control_lli.so"
         OLD_RPATH "/opt/ros/melodic/lib:/usr/lib/tumtools:/home/marco/ros/workspaces/final_project/src/build/devel/lib:"
         NEW_RPATH "/opt/ros/melodic/lib:/usr/lib/tumtools")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libmy_robot_control_lli.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/tum_ics_ur10_controller_tutorial/testSimpleEffortCtrl" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/tum_ics_ur10_controller_tutorial/testSimpleEffortCtrl")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/tum_ics_ur10_controller_tutorial/testSimpleEffortCtrl"
         RPATH "/opt/ros/melodic/lib:/usr/lib/tumtools")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/tum_ics_ur10_controller_tutorial" TYPE EXECUTABLE FILES "/home/marco/ros/workspaces/final_project/src/build/devel/lib/tum_ics_ur10_controller_tutorial/testSimpleEffortCtrl")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/tum_ics_ur10_controller_tutorial/testSimpleEffortCtrl" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/tum_ics_ur10_controller_tutorial/testSimpleEffortCtrl")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/tum_ics_ur10_controller_tutorial/testSimpleEffortCtrl"
         OLD_RPATH "/home/marco/ros/workspaces/final_project/src/build/devel/lib:/opt/ros/melodic/lib:/usr/lib/tumtools:"
         NEW_RPATH "/opt/ros/melodic/lib:/usr/lib/tumtools")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/tum_ics_ur10_controller_tutorial/testSimpleEffortCtrl")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tum_ics_ur10_controller_tutorial/launch" TYPE DIRECTORY FILES "/home/marco/ros/workspaces/final_project/src/tum_ics_ur10_controller_tutorial/launch/")
endif()

