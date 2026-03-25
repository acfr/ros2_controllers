// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ros2_controllers_interfaces:msg/SwerveControllerStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ros2_controllers_interfaces/msg/swerve_controller_status.hpp"


#ifndef ROS2_CONTROLLERS_INTERFACES__MSG__DETAIL__SWERVE_CONTROLLER_STATUS__TRAITS_HPP_
#define ROS2_CONTROLLERS_INTERFACES__MSG__DETAIL__SWERVE_CONTROLLER_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ros2_controllers_interfaces/msg/detail/swerve_controller_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace ros2_controllers_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const SwerveControllerStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: wheel_drive_position
  {
    if (msg.wheel_drive_position.size() == 0) {
      out << "wheel_drive_position: []";
    } else {
      out << "wheel_drive_position: [";
      size_t pending_items = msg.wheel_drive_position.size();
      for (auto item : msg.wheel_drive_position) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: wheel_drive_velocity
  {
    if (msg.wheel_drive_velocity.size() == 0) {
      out << "wheel_drive_velocity: []";
    } else {
      out << "wheel_drive_velocity: [";
      size_t pending_items = msg.wheel_drive_velocity.size();
      for (auto item : msg.wheel_drive_velocity) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: wheel_steer_position
  {
    if (msg.wheel_steer_position.size() == 0) {
      out << "wheel_steer_position: []";
    } else {
      out << "wheel_steer_position: [";
      size_t pending_items = msg.wheel_steer_position.size();
      for (auto item : msg.wheel_steer_position) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: wheel_icr
  {
    if (msg.wheel_icr.size() == 0) {
      out << "wheel_icr: []";
    } else {
      out << "wheel_icr: [";
      size_t pending_items = msg.wheel_icr.size();
      for (auto item : msg.wheel_icr) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: wheel_steering_angle_cmd
  {
    if (msg.wheel_steering_angle_cmd.size() == 0) {
      out << "wheel_steering_angle_cmd: []";
    } else {
      out << "wheel_steering_angle_cmd: [";
      size_t pending_items = msg.wheel_steering_angle_cmd.size();
      for (auto item : msg.wheel_steering_angle_cmd) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: wheel_drive_velocity_cmd
  {
    if (msg.wheel_drive_velocity_cmd.size() == 0) {
      out << "wheel_drive_velocity_cmd: []";
    } else {
      out << "wheel_drive_velocity_cmd: [";
      size_t pending_items = msg.wheel_drive_velocity_cmd.size();
      for (auto item : msg.wheel_drive_velocity_cmd) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SwerveControllerStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: wheel_drive_position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.wheel_drive_position.size() == 0) {
      out << "wheel_drive_position: []\n";
    } else {
      out << "wheel_drive_position:\n";
      for (auto item : msg.wheel_drive_position) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: wheel_drive_velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.wheel_drive_velocity.size() == 0) {
      out << "wheel_drive_velocity: []\n";
    } else {
      out << "wheel_drive_velocity:\n";
      for (auto item : msg.wheel_drive_velocity) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: wheel_steer_position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.wheel_steer_position.size() == 0) {
      out << "wheel_steer_position: []\n";
    } else {
      out << "wheel_steer_position:\n";
      for (auto item : msg.wheel_steer_position) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: wheel_icr
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.wheel_icr.size() == 0) {
      out << "wheel_icr: []\n";
    } else {
      out << "wheel_icr:\n";
      for (auto item : msg.wheel_icr) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: wheel_steering_angle_cmd
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.wheel_steering_angle_cmd.size() == 0) {
      out << "wheel_steering_angle_cmd: []\n";
    } else {
      out << "wheel_steering_angle_cmd:\n";
      for (auto item : msg.wheel_steering_angle_cmd) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: wheel_drive_velocity_cmd
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.wheel_drive_velocity_cmd.size() == 0) {
      out << "wheel_drive_velocity_cmd: []\n";
    } else {
      out << "wheel_drive_velocity_cmd:\n";
      for (auto item : msg.wheel_drive_velocity_cmd) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SwerveControllerStatus & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace ros2_controllers_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use ros2_controllers_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ros2_controllers_interfaces::msg::SwerveControllerStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  ros2_controllers_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ros2_controllers_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const ros2_controllers_interfaces::msg::SwerveControllerStatus & msg)
{
  return ros2_controllers_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ros2_controllers_interfaces::msg::SwerveControllerStatus>()
{
  return "ros2_controllers_interfaces::msg::SwerveControllerStatus";
}

template<>
inline const char * name<ros2_controllers_interfaces::msg::SwerveControllerStatus>()
{
  return "ros2_controllers_interfaces/msg/SwerveControllerStatus";
}

template<>
struct has_fixed_size<ros2_controllers_interfaces::msg::SwerveControllerStatus>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ros2_controllers_interfaces::msg::SwerveControllerStatus>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<ros2_controllers_interfaces::msg::SwerveControllerStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROS2_CONTROLLERS_INTERFACES__MSG__DETAIL__SWERVE_CONTROLLER_STATUS__TRAITS_HPP_
