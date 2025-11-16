cmake_minimum_required(VERSION 3.16)
project(swerve_controller)

find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
export_windows_symbols()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_msgs
  control_toolbox
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
add_compile_definitions(RCPPUTILS_VERSION_MAJOR=${rcpputils_VERSION_MAJOR})
add_compile_definitions(RCPPUTILS_VERSION_MINOR=${rcpputils_VERSION_MINOR})

get_target_property(TB_INCLUDE_DIRS control_toolbox::rate_limiter_parameters INTERFACE_INCLUDE_DIRECTORIES)
generate_parameter_library(swerve_controller_parameters
  src/swerve_controller_parameter.yaml
  ${TB_INCLUDE_DIRS}/control_toolbox/custom_validators.hpp
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
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/swerve_controller>
)

target_link_libraries(swerve_controller   
  PUBLIC
    swerve_controller_parameters
    control_toolbox::rate_limiter_parameters
    control_toolbox::control_toolbox
    controller_interface::controller_interface
    hardware_interface::hardware_interface
    angles::angles
    pluginlib::pluginlib
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    rcpputils::rcpputils
    realtime_tools::realtime_tools
    tf2::tf2
    ${tf2_geometry_msgs_TARGETS}
    ${std_srvs_TARGETS}
    ${tf2_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${visualization_msgs_TARGETS}
    ${ros2_controllers_interfaces_TARGETS})
pluginlib_export_plugin_description_file(controller_interface swerve_controller.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(hardware_interface REQUIRED)

  add_definitions(-DTEST_FILES_DIRECTORY="${CMAKE_CURRENT_SOURCE_DIR}/test")
  ament_add_gmock(test_swerve_controller
    test/test_swerve_controller.cpp)
  target_link_libraries(test_swerve_controller
    swerve_controller
  )
  
  ament_add_gmock(test_load_swerve_controller test/test_load_swerve_controller.cpp)
  target_link_libraries(test_load_swerve_controller
    controller_manager::controller_manager
  )
endif()

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
