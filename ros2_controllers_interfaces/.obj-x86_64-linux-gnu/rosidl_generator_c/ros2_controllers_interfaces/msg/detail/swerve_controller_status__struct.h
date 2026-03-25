// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros2_controllers_interfaces:msg/SwerveControllerStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ros2_controllers_interfaces/msg/swerve_controller_status.h"


#ifndef ROS2_CONTROLLERS_INTERFACES__MSG__DETAIL__SWERVE_CONTROLLER_STATUS__STRUCT_H_
#define ROS2_CONTROLLERS_INTERFACES__MSG__DETAIL__SWERVE_CONTROLLER_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'wheel_drive_position'
// Member 'wheel_drive_velocity'
// Member 'wheel_steer_position'
// Member 'wheel_icr'
// Member 'wheel_steering_angle_cmd'
// Member 'wheel_drive_velocity_cmd'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/SwerveControllerStatus in the package ros2_controllers_interfaces.
typedef struct ros2_controllers_interfaces__msg__SwerveControllerStatus
{
  std_msgs__msg__Header header;
  /// positions of wheel drive joint if the robot is controlled by position
  rosidl_runtime_c__double__Sequence wheel_drive_position;
  /// positions of wheel drive joint if the robot is controlled by velocity
  rosidl_runtime_c__double__Sequence wheel_drive_velocity;
  /// positions of steering joints
  rosidl_runtime_c__double__Sequence wheel_steer_position;
  rosidl_runtime_c__double__Sequence wheel_icr;
  rosidl_runtime_c__double__Sequence wheel_steering_angle_cmd;
  rosidl_runtime_c__double__Sequence wheel_drive_velocity_cmd;
} ros2_controllers_interfaces__msg__SwerveControllerStatus;

// Struct for a sequence of ros2_controllers_interfaces__msg__SwerveControllerStatus.
typedef struct ros2_controllers_interfaces__msg__SwerveControllerStatus__Sequence
{
  ros2_controllers_interfaces__msg__SwerveControllerStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros2_controllers_interfaces__msg__SwerveControllerStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS2_CONTROLLERS_INTERFACES__MSG__DETAIL__SWERVE_CONTROLLER_STATUS__STRUCT_H_
