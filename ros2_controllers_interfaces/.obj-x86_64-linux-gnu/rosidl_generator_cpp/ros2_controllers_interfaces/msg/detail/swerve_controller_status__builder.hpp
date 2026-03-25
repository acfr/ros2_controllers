// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros2_controllers_interfaces:msg/SwerveControllerStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ros2_controllers_interfaces/msg/swerve_controller_status.hpp"


#ifndef ROS2_CONTROLLERS_INTERFACES__MSG__DETAIL__SWERVE_CONTROLLER_STATUS__BUILDER_HPP_
#define ROS2_CONTROLLERS_INTERFACES__MSG__DETAIL__SWERVE_CONTROLLER_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros2_controllers_interfaces/msg/detail/swerve_controller_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros2_controllers_interfaces
{

namespace msg
{

namespace builder
{

class Init_SwerveControllerStatus_wheel_drive_velocity_cmd
{
public:
  explicit Init_SwerveControllerStatus_wheel_drive_velocity_cmd(::ros2_controllers_interfaces::msg::SwerveControllerStatus & msg)
  : msg_(msg)
  {}
  ::ros2_controllers_interfaces::msg::SwerveControllerStatus wheel_drive_velocity_cmd(::ros2_controllers_interfaces::msg::SwerveControllerStatus::_wheel_drive_velocity_cmd_type arg)
  {
    msg_.wheel_drive_velocity_cmd = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros2_controllers_interfaces::msg::SwerveControllerStatus msg_;
};

class Init_SwerveControllerStatus_wheel_steering_angle_cmd
{
public:
  explicit Init_SwerveControllerStatus_wheel_steering_angle_cmd(::ros2_controllers_interfaces::msg::SwerveControllerStatus & msg)
  : msg_(msg)
  {}
  Init_SwerveControllerStatus_wheel_drive_velocity_cmd wheel_steering_angle_cmd(::ros2_controllers_interfaces::msg::SwerveControllerStatus::_wheel_steering_angle_cmd_type arg)
  {
    msg_.wheel_steering_angle_cmd = std::move(arg);
    return Init_SwerveControllerStatus_wheel_drive_velocity_cmd(msg_);
  }

private:
  ::ros2_controllers_interfaces::msg::SwerveControllerStatus msg_;
};

class Init_SwerveControllerStatus_wheel_icr
{
public:
  explicit Init_SwerveControllerStatus_wheel_icr(::ros2_controllers_interfaces::msg::SwerveControllerStatus & msg)
  : msg_(msg)
  {}
  Init_SwerveControllerStatus_wheel_steering_angle_cmd wheel_icr(::ros2_controllers_interfaces::msg::SwerveControllerStatus::_wheel_icr_type arg)
  {
    msg_.wheel_icr = std::move(arg);
    return Init_SwerveControllerStatus_wheel_steering_angle_cmd(msg_);
  }

private:
  ::ros2_controllers_interfaces::msg::SwerveControllerStatus msg_;
};

class Init_SwerveControllerStatus_wheel_steer_position
{
public:
  explicit Init_SwerveControllerStatus_wheel_steer_position(::ros2_controllers_interfaces::msg::SwerveControllerStatus & msg)
  : msg_(msg)
  {}
  Init_SwerveControllerStatus_wheel_icr wheel_steer_position(::ros2_controllers_interfaces::msg::SwerveControllerStatus::_wheel_steer_position_type arg)
  {
    msg_.wheel_steer_position = std::move(arg);
    return Init_SwerveControllerStatus_wheel_icr(msg_);
  }

private:
  ::ros2_controllers_interfaces::msg::SwerveControllerStatus msg_;
};

class Init_SwerveControllerStatus_wheel_drive_velocity
{
public:
  explicit Init_SwerveControllerStatus_wheel_drive_velocity(::ros2_controllers_interfaces::msg::SwerveControllerStatus & msg)
  : msg_(msg)
  {}
  Init_SwerveControllerStatus_wheel_steer_position wheel_drive_velocity(::ros2_controllers_interfaces::msg::SwerveControllerStatus::_wheel_drive_velocity_type arg)
  {
    msg_.wheel_drive_velocity = std::move(arg);
    return Init_SwerveControllerStatus_wheel_steer_position(msg_);
  }

private:
  ::ros2_controllers_interfaces::msg::SwerveControllerStatus msg_;
};

class Init_SwerveControllerStatus_wheel_drive_position
{
public:
  explicit Init_SwerveControllerStatus_wheel_drive_position(::ros2_controllers_interfaces::msg::SwerveControllerStatus & msg)
  : msg_(msg)
  {}
  Init_SwerveControllerStatus_wheel_drive_velocity wheel_drive_position(::ros2_controllers_interfaces::msg::SwerveControllerStatus::_wheel_drive_position_type arg)
  {
    msg_.wheel_drive_position = std::move(arg);
    return Init_SwerveControllerStatus_wheel_drive_velocity(msg_);
  }

private:
  ::ros2_controllers_interfaces::msg::SwerveControllerStatus msg_;
};

class Init_SwerveControllerStatus_header
{
public:
  Init_SwerveControllerStatus_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SwerveControllerStatus_wheel_drive_position header(::ros2_controllers_interfaces::msg::SwerveControllerStatus::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_SwerveControllerStatus_wheel_drive_position(msg_);
  }

private:
  ::ros2_controllers_interfaces::msg::SwerveControllerStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros2_controllers_interfaces::msg::SwerveControllerStatus>()
{
  return ros2_controllers_interfaces::msg::builder::Init_SwerveControllerStatus_header();
}

}  // namespace ros2_controllers_interfaces

#endif  // ROS2_CONTROLLERS_INTERFACES__MSG__DETAIL__SWERVE_CONTROLLER_STATUS__BUILDER_HPP_
