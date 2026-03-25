// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ros2_controllers_interfaces:msg/SwerveControllerStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ros2_controllers_interfaces/msg/swerve_controller_status.hpp"


#ifndef ROS2_CONTROLLERS_INTERFACES__MSG__DETAIL__SWERVE_CONTROLLER_STATUS__STRUCT_HPP_
#define ROS2_CONTROLLERS_INTERFACES__MSG__DETAIL__SWERVE_CONTROLLER_STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ros2_controllers_interfaces__msg__SwerveControllerStatus __attribute__((deprecated))
#else
# define DEPRECATED__ros2_controllers_interfaces__msg__SwerveControllerStatus __declspec(deprecated)
#endif

namespace ros2_controllers_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SwerveControllerStatus_
{
  using Type = SwerveControllerStatus_<ContainerAllocator>;

  explicit SwerveControllerStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit SwerveControllerStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _wheel_drive_position_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _wheel_drive_position_type wheel_drive_position;
  using _wheel_drive_velocity_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _wheel_drive_velocity_type wheel_drive_velocity;
  using _wheel_steer_position_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _wheel_steer_position_type wheel_steer_position;
  using _wheel_icr_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _wheel_icr_type wheel_icr;
  using _wheel_steering_angle_cmd_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _wheel_steering_angle_cmd_type wheel_steering_angle_cmd;
  using _wheel_drive_velocity_cmd_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _wheel_drive_velocity_cmd_type wheel_drive_velocity_cmd;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__wheel_drive_position(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->wheel_drive_position = _arg;
    return *this;
  }
  Type & set__wheel_drive_velocity(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->wheel_drive_velocity = _arg;
    return *this;
  }
  Type & set__wheel_steer_position(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->wheel_steer_position = _arg;
    return *this;
  }
  Type & set__wheel_icr(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->wheel_icr = _arg;
    return *this;
  }
  Type & set__wheel_steering_angle_cmd(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->wheel_steering_angle_cmd = _arg;
    return *this;
  }
  Type & set__wheel_drive_velocity_cmd(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->wheel_drive_velocity_cmd = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ros2_controllers_interfaces::msg::SwerveControllerStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const ros2_controllers_interfaces::msg::SwerveControllerStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ros2_controllers_interfaces::msg::SwerveControllerStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ros2_controllers_interfaces::msg::SwerveControllerStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ros2_controllers_interfaces::msg::SwerveControllerStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ros2_controllers_interfaces::msg::SwerveControllerStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ros2_controllers_interfaces::msg::SwerveControllerStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ros2_controllers_interfaces::msg::SwerveControllerStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ros2_controllers_interfaces::msg::SwerveControllerStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ros2_controllers_interfaces::msg::SwerveControllerStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ros2_controllers_interfaces__msg__SwerveControllerStatus
    std::shared_ptr<ros2_controllers_interfaces::msg::SwerveControllerStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ros2_controllers_interfaces__msg__SwerveControllerStatus
    std::shared_ptr<ros2_controllers_interfaces::msg::SwerveControllerStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SwerveControllerStatus_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->wheel_drive_position != other.wheel_drive_position) {
      return false;
    }
    if (this->wheel_drive_velocity != other.wheel_drive_velocity) {
      return false;
    }
    if (this->wheel_steer_position != other.wheel_steer_position) {
      return false;
    }
    if (this->wheel_icr != other.wheel_icr) {
      return false;
    }
    if (this->wheel_steering_angle_cmd != other.wheel_steering_angle_cmd) {
      return false;
    }
    if (this->wheel_drive_velocity_cmd != other.wheel_drive_velocity_cmd) {
      return false;
    }
    return true;
  }
  bool operator!=(const SwerveControllerStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SwerveControllerStatus_

// alias to use template instance with default allocator
using SwerveControllerStatus =
  ros2_controllers_interfaces::msg::SwerveControllerStatus_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ros2_controllers_interfaces

#endif  // ROS2_CONTROLLERS_INTERFACES__MSG__DETAIL__SWERVE_CONTROLLER_STATUS__STRUCT_HPP_
