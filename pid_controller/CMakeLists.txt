cmake_minimum_required(VERSION 3.16)
project(pid_controller LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

if(WIN32)
  add_compile_definitions(
    # For math constants
    _USE_MATH_DEFINES
    # Minimize Windows namespace collision
    NOMINMAX
    WIN32_LEAN_AND_MEAN
  )
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  angles
  control_msgs
  control_toolbox
  controller_interface
  generate_parameter_library
  hardware_interface
  parameter_traits
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_srvs
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(pid_controller_parameters
  src/pid_controller.yaml
)

add_library(pid_controller SHARED
  src/pid_controller.cpp
)
target_compile_features(pid_controller PUBLIC cxx_std_17)
target_include_directories(pid_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/pid_controller>
)
target_link_libraries(pid_controller PUBLIC
  pid_controller_parameters
)
ament_target_dependencies(pid_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(pid_controller PRIVATE "PID_CONTROLLER_BUILDING_DLL")

pluginlib_export_plugin_description_file(controller_interface pid_controller.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  add_rostest_with_parameters_gmock(
    test_pid_controller
    test/test_pid_controller.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test/pid_controller_params.yaml
  )
  target_include_directories(test_pid_controller PRIVATE include)
  target_link_libraries(test_pid_controller pid_controller)
  ament_target_dependencies(
    test_pid_controller
    controller_interface
    hardware_interface
  )

  add_rostest_with_parameters_gmock(
    test_pid_controller_preceding
    test/test_pid_controller_preceding.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test/pid_controller_preceding_params.yaml
  )
  target_include_directories(test_pid_controller_preceding PRIVATE include)
  target_link_libraries(test_pid_controller_preceding pid_controller)
  ament_target_dependencies(
    test_pid_controller_preceding
    controller_interface
    hardware_interface
  )

  ament_add_gmock(test_load_pid_controller test/test_load_pid_controller.cpp)
  target_include_directories(test_load_pid_controller PRIVATE include)
  ament_target_dependencies(
    test_load_pid_controller
    controller_manager
    ros2_control_test_assets
  )
endif()

install(
  DIRECTORY include/
  DESTINATION include/pid_controller
)

install(TARGETS
    pid_controller
    pid_controller_parameters
  EXPORT export_pid_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_pid_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
