# Install script for directory: /home/steven/ros2_ws/src/command_endeffector_cartesian/src/ur5_endeffector_cartesian

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/steven/ros2_ws/src/command_endeffector_cartesian/install/ur5_endeffector_cartesian")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
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

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur5_endeffector_cartesian/ur5_ros2_cartesian" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur5_endeffector_cartesian/ur5_ros2_cartesian")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur5_endeffector_cartesian/ur5_ros2_cartesian"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/ur5_endeffector_cartesian" TYPE EXECUTABLE FILES "/home/steven/ros2_ws/src/command_endeffector_cartesian/build/ur5_endeffector_cartesian/ur5_ros2_cartesian")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur5_endeffector_cartesian/ur5_ros2_cartesian" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur5_endeffector_cartesian/ur5_ros2_cartesian")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur5_endeffector_cartesian/ur5_ros2_cartesian"
         OLD_RPATH "/home/steven/ws_moveit2/install/moveit_ros_planning_interface/lib:/home/steven/ws_moveit2/install/moveit_visual_tools/lib:/home/steven/ws_moveit2/install/moveit_ros_move_group/lib:/home/steven/ws_moveit2/install/moveit_core/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib:/home/steven/ws_moveit2/install/moveit_ros_warehouse/lib:/home/steven/ws_moveit2/install/moveit_ros_planning/lib:/home/steven/ws_moveit2/install/moveit_ros_occupancy_map_monitor/lib:/home/steven/ws_moveit2/install/srdfdom/lib:/opt/ros/humble/opt/rviz_ogre_vendor/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ur5_endeffector_cartesian/ur5_ros2_cartesian")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur5_endeffector_cartesian" TYPE DIRECTORY FILES "/home/steven/ros2_ws/src/command_endeffector_cartesian/src/ur5_endeffector_cartesian/launch")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/steven/ros2_ws/src/command_endeffector_cartesian/build/ur5_endeffector_cartesian/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/ur5_endeffector_cartesian")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/steven/ros2_ws/src/command_endeffector_cartesian/build/ur5_endeffector_cartesian/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/ur5_endeffector_cartesian")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur5_endeffector_cartesian/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur5_endeffector_cartesian/environment" TYPE FILE FILES "/home/steven/ros2_ws/src/command_endeffector_cartesian/build/ur5_endeffector_cartesian/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur5_endeffector_cartesian/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur5_endeffector_cartesian/environment" TYPE FILE FILES "/home/steven/ros2_ws/src/command_endeffector_cartesian/build/ur5_endeffector_cartesian/ament_cmake_environment_hooks/path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur5_endeffector_cartesian" TYPE FILE FILES "/home/steven/ros2_ws/src/command_endeffector_cartesian/build/ur5_endeffector_cartesian/ament_cmake_environment_hooks/local_setup.bash")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur5_endeffector_cartesian" TYPE FILE FILES "/home/steven/ros2_ws/src/command_endeffector_cartesian/build/ur5_endeffector_cartesian/ament_cmake_environment_hooks/local_setup.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur5_endeffector_cartesian" TYPE FILE FILES "/home/steven/ros2_ws/src/command_endeffector_cartesian/build/ur5_endeffector_cartesian/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur5_endeffector_cartesian" TYPE FILE FILES "/home/steven/ros2_ws/src/command_endeffector_cartesian/build/ur5_endeffector_cartesian/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur5_endeffector_cartesian" TYPE FILE FILES "/home/steven/ros2_ws/src/command_endeffector_cartesian/build/ur5_endeffector_cartesian/ament_cmake_environment_hooks/package.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/steven/ros2_ws/src/command_endeffector_cartesian/build/ur5_endeffector_cartesian/ament_cmake_index/share/ament_index/resource_index/packages/ur5_endeffector_cartesian")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur5_endeffector_cartesian/cmake" TYPE FILE FILES
    "/home/steven/ros2_ws/src/command_endeffector_cartesian/build/ur5_endeffector_cartesian/ament_cmake_core/ur5_endeffector_cartesianConfig.cmake"
    "/home/steven/ros2_ws/src/command_endeffector_cartesian/build/ur5_endeffector_cartesian/ament_cmake_core/ur5_endeffector_cartesianConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur5_endeffector_cartesian" TYPE FILE FILES "/home/steven/ros2_ws/src/command_endeffector_cartesian/src/ur5_endeffector_cartesian/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/steven/ros2_ws/src/command_endeffector_cartesian/build/ur5_endeffector_cartesian/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
