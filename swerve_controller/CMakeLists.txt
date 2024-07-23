cmake_minimum_required(VERSION 3.16)
project(swerve_controller LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_msgs
  controller_interface
  generate_parameter_library
  geometry_msgs
  hardware_interface
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rcpputils
  realtime_tools
  std_srvs
  tf2
  tf2_msgs
  tf2_geometry_msgs
  angles
  visualization_msgs
  ros2_controllers_interfaces
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(${THIS_PACKAGE_INCLUDE_DIRS})

generate_parameter_library(swerve_controller_parameters
  src/swerve_controller_parameter.yaml
)

add_library(
  swerve_controller
  SHARED
  src/swerve_controller.cpp
  src/odometry.cpp
  src/speed_limiter.cpp
)
target_compile_features(swerve_controller PUBLIC cxx_std_17)
target_include_directories(swerve_controller PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/swerve_controller>")
target_link_libraries(swerve_controller PUBLIC
  swerve_controller_parameters)
ament_target_dependencies(swerve_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(swerve_controller PRIVATE "STEERING_CONTROLLERS_BUILDING_DLL")
pluginlib_export_plugin_description_file(controller_interface swerve_controller.xml)

install(
  DIRECTORY include/
  DESTINATION include/swerve_controller
)

install(
  TARGETS swerve_controller swerve_controller_parameters
  EXPORT export_swerve_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_swerve_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
