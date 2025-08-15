cmake_minimum_required(VERSION 3.16)
project(itri_arm)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(controller_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_msgs
  controller_interface
  controller_manager
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  sensor_msgs
  std_msgs
)

# COMPILE
add_library(
  itri_arm_hardware
  SHARED
  src/itri_arm_hardware.cpp
)
target_compile_features(itri_arm_hardware PUBLIC cxx_std_17)
target_include_directories(itri_arm_hardware PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/itri_arm>
)
ament_target_dependencies(
  itri_arm_hardware PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(itri_arm_hardware PRIVATE "ITRI_ARM_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface itri_arm_hardware.xml)

# INSTALL
install(
  DIRECTORY include/
  DESTINATION include/itri_arm
)
install(
  TARGETS itri_arm_hardware
  EXPORT export_itri_arm
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY config launch urdf
  DESTINATION share/itri_arm
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

## EXPORTS
ament_export_targets(export_itri_arm HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()