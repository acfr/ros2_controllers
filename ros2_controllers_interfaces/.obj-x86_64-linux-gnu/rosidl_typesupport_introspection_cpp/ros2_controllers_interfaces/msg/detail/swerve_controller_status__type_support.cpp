// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from ros2_controllers_interfaces:msg/SwerveControllerStatus.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "ros2_controllers_interfaces/msg/detail/swerve_controller_status__functions.h"
#include "ros2_controllers_interfaces/msg/detail/swerve_controller_status__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace ros2_controllers_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void SwerveControllerStatus_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) ros2_controllers_interfaces::msg::SwerveControllerStatus(_init);
}

void SwerveControllerStatus_fini_function(void * message_memory)
{
  auto typed_message = static_cast<ros2_controllers_interfaces::msg::SwerveControllerStatus *>(message_memory);
  typed_message->~SwerveControllerStatus();
}

size_t size_function__SwerveControllerStatus__wheel_drive_position(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__SwerveControllerStatus__wheel_drive_position(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__SwerveControllerStatus__wheel_drive_position(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__SwerveControllerStatus__wheel_drive_position(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__SwerveControllerStatus__wheel_drive_position(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__SwerveControllerStatus__wheel_drive_position(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__SwerveControllerStatus__wheel_drive_position(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__SwerveControllerStatus__wheel_drive_position(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

size_t size_function__SwerveControllerStatus__wheel_drive_velocity(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__SwerveControllerStatus__wheel_drive_velocity(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__SwerveControllerStatus__wheel_drive_velocity(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__SwerveControllerStatus__wheel_drive_velocity(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__SwerveControllerStatus__wheel_drive_velocity(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__SwerveControllerStatus__wheel_drive_velocity(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__SwerveControllerStatus__wheel_drive_velocity(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__SwerveControllerStatus__wheel_drive_velocity(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

size_t size_function__SwerveControllerStatus__wheel_steer_position(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__SwerveControllerStatus__wheel_steer_position(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__SwerveControllerStatus__wheel_steer_position(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__SwerveControllerStatus__wheel_steer_position(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__SwerveControllerStatus__wheel_steer_position(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__SwerveControllerStatus__wheel_steer_position(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__SwerveControllerStatus__wheel_steer_position(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__SwerveControllerStatus__wheel_steer_position(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

size_t size_function__SwerveControllerStatus__wheel_icr(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__SwerveControllerStatus__wheel_icr(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__SwerveControllerStatus__wheel_icr(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__SwerveControllerStatus__wheel_icr(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__SwerveControllerStatus__wheel_icr(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__SwerveControllerStatus__wheel_icr(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__SwerveControllerStatus__wheel_icr(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__SwerveControllerStatus__wheel_icr(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

size_t size_function__SwerveControllerStatus__wheel_steering_angle_cmd(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__SwerveControllerStatus__wheel_steering_angle_cmd(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__SwerveControllerStatus__wheel_steering_angle_cmd(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__SwerveControllerStatus__wheel_steering_angle_cmd(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__SwerveControllerStatus__wheel_steering_angle_cmd(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__SwerveControllerStatus__wheel_steering_angle_cmd(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__SwerveControllerStatus__wheel_steering_angle_cmd(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__SwerveControllerStatus__wheel_steering_angle_cmd(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

size_t size_function__SwerveControllerStatus__wheel_drive_velocity_cmd(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__SwerveControllerStatus__wheel_drive_velocity_cmd(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__SwerveControllerStatus__wheel_drive_velocity_cmd(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__SwerveControllerStatus__wheel_drive_velocity_cmd(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__SwerveControllerStatus__wheel_drive_velocity_cmd(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__SwerveControllerStatus__wheel_drive_velocity_cmd(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__SwerveControllerStatus__wheel_drive_velocity_cmd(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__SwerveControllerStatus__wheel_drive_velocity_cmd(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember SwerveControllerStatus_message_member_array[7] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_controllers_interfaces::msg::SwerveControllerStatus, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "wheel_drive_position",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_controllers_interfaces::msg::SwerveControllerStatus, wheel_drive_position),  // bytes offset in struct
    nullptr,  // default value
    size_function__SwerveControllerStatus__wheel_drive_position,  // size() function pointer
    get_const_function__SwerveControllerStatus__wheel_drive_position,  // get_const(index) function pointer
    get_function__SwerveControllerStatus__wheel_drive_position,  // get(index) function pointer
    fetch_function__SwerveControllerStatus__wheel_drive_position,  // fetch(index, &value) function pointer
    assign_function__SwerveControllerStatus__wheel_drive_position,  // assign(index, value) function pointer
    resize_function__SwerveControllerStatus__wheel_drive_position  // resize(index) function pointer
  },
  {
    "wheel_drive_velocity",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_controllers_interfaces::msg::SwerveControllerStatus, wheel_drive_velocity),  // bytes offset in struct
    nullptr,  // default value
    size_function__SwerveControllerStatus__wheel_drive_velocity,  // size() function pointer
    get_const_function__SwerveControllerStatus__wheel_drive_velocity,  // get_const(index) function pointer
    get_function__SwerveControllerStatus__wheel_drive_velocity,  // get(index) function pointer
    fetch_function__SwerveControllerStatus__wheel_drive_velocity,  // fetch(index, &value) function pointer
    assign_function__SwerveControllerStatus__wheel_drive_velocity,  // assign(index, value) function pointer
    resize_function__SwerveControllerStatus__wheel_drive_velocity  // resize(index) function pointer
  },
  {
    "wheel_steer_position",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_controllers_interfaces::msg::SwerveControllerStatus, wheel_steer_position),  // bytes offset in struct
    nullptr,  // default value
    size_function__SwerveControllerStatus__wheel_steer_position,  // size() function pointer
    get_const_function__SwerveControllerStatus__wheel_steer_position,  // get_const(index) function pointer
    get_function__SwerveControllerStatus__wheel_steer_position,  // get(index) function pointer
    fetch_function__SwerveControllerStatus__wheel_steer_position,  // fetch(index, &value) function pointer
    assign_function__SwerveControllerStatus__wheel_steer_position,  // assign(index, value) function pointer
    resize_function__SwerveControllerStatus__wheel_steer_position  // resize(index) function pointer
  },
  {
    "wheel_icr",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_controllers_interfaces::msg::SwerveControllerStatus, wheel_icr),  // bytes offset in struct
    nullptr,  // default value
    size_function__SwerveControllerStatus__wheel_icr,  // size() function pointer
    get_const_function__SwerveControllerStatus__wheel_icr,  // get_const(index) function pointer
    get_function__SwerveControllerStatus__wheel_icr,  // get(index) function pointer
    fetch_function__SwerveControllerStatus__wheel_icr,  // fetch(index, &value) function pointer
    assign_function__SwerveControllerStatus__wheel_icr,  // assign(index, value) function pointer
    resize_function__SwerveControllerStatus__wheel_icr  // resize(index) function pointer
  },
  {
    "wheel_steering_angle_cmd",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_controllers_interfaces::msg::SwerveControllerStatus, wheel_steering_angle_cmd),  // bytes offset in struct
    nullptr,  // default value
    size_function__SwerveControllerStatus__wheel_steering_angle_cmd,  // size() function pointer
    get_const_function__SwerveControllerStatus__wheel_steering_angle_cmd,  // get_const(index) function pointer
    get_function__SwerveControllerStatus__wheel_steering_angle_cmd,  // get(index) function pointer
    fetch_function__SwerveControllerStatus__wheel_steering_angle_cmd,  // fetch(index, &value) function pointer
    assign_function__SwerveControllerStatus__wheel_steering_angle_cmd,  // assign(index, value) function pointer
    resize_function__SwerveControllerStatus__wheel_steering_angle_cmd  // resize(index) function pointer
  },
  {
    "wheel_drive_velocity_cmd",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_controllers_interfaces::msg::SwerveControllerStatus, wheel_drive_velocity_cmd),  // bytes offset in struct
    nullptr,  // default value
    size_function__SwerveControllerStatus__wheel_drive_velocity_cmd,  // size() function pointer
    get_const_function__SwerveControllerStatus__wheel_drive_velocity_cmd,  // get_const(index) function pointer
    get_function__SwerveControllerStatus__wheel_drive_velocity_cmd,  // get(index) function pointer
    fetch_function__SwerveControllerStatus__wheel_drive_velocity_cmd,  // fetch(index, &value) function pointer
    assign_function__SwerveControllerStatus__wheel_drive_velocity_cmd,  // assign(index, value) function pointer
    resize_function__SwerveControllerStatus__wheel_drive_velocity_cmd  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers SwerveControllerStatus_message_members = {
  "ros2_controllers_interfaces::msg",  // message namespace
  "SwerveControllerStatus",  // message name
  7,  // number of fields
  sizeof(ros2_controllers_interfaces::msg::SwerveControllerStatus),
  false,  // has_any_key_member_
  SwerveControllerStatus_message_member_array,  // message members
  SwerveControllerStatus_init_function,  // function to initialize message memory (memory has to be allocated)
  SwerveControllerStatus_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t SwerveControllerStatus_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &SwerveControllerStatus_message_members,
  get_message_typesupport_handle_function,
  &ros2_controllers_interfaces__msg__SwerveControllerStatus__get_type_hash,
  &ros2_controllers_interfaces__msg__SwerveControllerStatus__get_type_description,
  &ros2_controllers_interfaces__msg__SwerveControllerStatus__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace ros2_controllers_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<ros2_controllers_interfaces::msg::SwerveControllerStatus>()
{
  return &::ros2_controllers_interfaces::msg::rosidl_typesupport_introspection_cpp::SwerveControllerStatus_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, ros2_controllers_interfaces, msg, SwerveControllerStatus)() {
  return &::ros2_controllers_interfaces::msg::rosidl_typesupport_introspection_cpp::SwerveControllerStatus_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
