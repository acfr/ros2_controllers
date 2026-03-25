// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from ros2_controllers_interfaces:msg/SwerveControllerStatus.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "ros2_controllers_interfaces/msg/detail/swerve_controller_status__rosidl_typesupport_introspection_c.h"
#include "ros2_controllers_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "ros2_controllers_interfaces/msg/detail/swerve_controller_status__functions.h"
#include "ros2_controllers_interfaces/msg/detail/swerve_controller_status__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `wheel_drive_position`
// Member `wheel_drive_velocity`
// Member `wheel_steer_position`
// Member `wheel_icr`
// Member `wheel_steering_angle_cmd`
// Member `wheel_drive_velocity_cmd`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__SwerveControllerStatus_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  ros2_controllers_interfaces__msg__SwerveControllerStatus__init(message_memory);
}

void ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__SwerveControllerStatus_fini_function(void * message_memory)
{
  ros2_controllers_interfaces__msg__SwerveControllerStatus__fini(message_memory);
}

size_t ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__size_function__SwerveControllerStatus__wheel_drive_position(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__get_const_function__SwerveControllerStatus__wheel_drive_position(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__get_function__SwerveControllerStatus__wheel_drive_position(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__fetch_function__SwerveControllerStatus__wheel_drive_position(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__get_const_function__SwerveControllerStatus__wheel_drive_position(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__assign_function__SwerveControllerStatus__wheel_drive_position(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__get_function__SwerveControllerStatus__wheel_drive_position(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__resize_function__SwerveControllerStatus__wheel_drive_position(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__size_function__SwerveControllerStatus__wheel_drive_velocity(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__get_const_function__SwerveControllerStatus__wheel_drive_velocity(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__get_function__SwerveControllerStatus__wheel_drive_velocity(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__fetch_function__SwerveControllerStatus__wheel_drive_velocity(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__get_const_function__SwerveControllerStatus__wheel_drive_velocity(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__assign_function__SwerveControllerStatus__wheel_drive_velocity(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__get_function__SwerveControllerStatus__wheel_drive_velocity(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__resize_function__SwerveControllerStatus__wheel_drive_velocity(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__size_function__SwerveControllerStatus__wheel_steer_position(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__get_const_function__SwerveControllerStatus__wheel_steer_position(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__get_function__SwerveControllerStatus__wheel_steer_position(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__fetch_function__SwerveControllerStatus__wheel_steer_position(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__get_const_function__SwerveControllerStatus__wheel_steer_position(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__assign_function__SwerveControllerStatus__wheel_steer_position(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__get_function__SwerveControllerStatus__wheel_steer_position(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__resize_function__SwerveControllerStatus__wheel_steer_position(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__size_function__SwerveControllerStatus__wheel_icr(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__get_const_function__SwerveControllerStatus__wheel_icr(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__get_function__SwerveControllerStatus__wheel_icr(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__fetch_function__SwerveControllerStatus__wheel_icr(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__get_const_function__SwerveControllerStatus__wheel_icr(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__assign_function__SwerveControllerStatus__wheel_icr(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__get_function__SwerveControllerStatus__wheel_icr(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__resize_function__SwerveControllerStatus__wheel_icr(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__size_function__SwerveControllerStatus__wheel_steering_angle_cmd(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__get_const_function__SwerveControllerStatus__wheel_steering_angle_cmd(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__get_function__SwerveControllerStatus__wheel_steering_angle_cmd(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__fetch_function__SwerveControllerStatus__wheel_steering_angle_cmd(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__get_const_function__SwerveControllerStatus__wheel_steering_angle_cmd(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__assign_function__SwerveControllerStatus__wheel_steering_angle_cmd(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__get_function__SwerveControllerStatus__wheel_steering_angle_cmd(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__resize_function__SwerveControllerStatus__wheel_steering_angle_cmd(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__size_function__SwerveControllerStatus__wheel_drive_velocity_cmd(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__get_const_function__SwerveControllerStatus__wheel_drive_velocity_cmd(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__get_function__SwerveControllerStatus__wheel_drive_velocity_cmd(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__fetch_function__SwerveControllerStatus__wheel_drive_velocity_cmd(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__get_const_function__SwerveControllerStatus__wheel_drive_velocity_cmd(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__assign_function__SwerveControllerStatus__wheel_drive_velocity_cmd(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__get_function__SwerveControllerStatus__wheel_drive_velocity_cmd(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__resize_function__SwerveControllerStatus__wheel_drive_velocity_cmd(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__SwerveControllerStatus_message_member_array[7] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_controllers_interfaces__msg__SwerveControllerStatus, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "wheel_drive_position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_controllers_interfaces__msg__SwerveControllerStatus, wheel_drive_position),  // bytes offset in struct
    NULL,  // default value
    ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__size_function__SwerveControllerStatus__wheel_drive_position,  // size() function pointer
    ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__get_const_function__SwerveControllerStatus__wheel_drive_position,  // get_const(index) function pointer
    ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__get_function__SwerveControllerStatus__wheel_drive_position,  // get(index) function pointer
    ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__fetch_function__SwerveControllerStatus__wheel_drive_position,  // fetch(index, &value) function pointer
    ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__assign_function__SwerveControllerStatus__wheel_drive_position,  // assign(index, value) function pointer
    ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__resize_function__SwerveControllerStatus__wheel_drive_position  // resize(index) function pointer
  },
  {
    "wheel_drive_velocity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_controllers_interfaces__msg__SwerveControllerStatus, wheel_drive_velocity),  // bytes offset in struct
    NULL,  // default value
    ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__size_function__SwerveControllerStatus__wheel_drive_velocity,  // size() function pointer
    ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__get_const_function__SwerveControllerStatus__wheel_drive_velocity,  // get_const(index) function pointer
    ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__get_function__SwerveControllerStatus__wheel_drive_velocity,  // get(index) function pointer
    ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__fetch_function__SwerveControllerStatus__wheel_drive_velocity,  // fetch(index, &value) function pointer
    ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__assign_function__SwerveControllerStatus__wheel_drive_velocity,  // assign(index, value) function pointer
    ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__resize_function__SwerveControllerStatus__wheel_drive_velocity  // resize(index) function pointer
  },
  {
    "wheel_steer_position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_controllers_interfaces__msg__SwerveControllerStatus, wheel_steer_position),  // bytes offset in struct
    NULL,  // default value
    ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__size_function__SwerveControllerStatus__wheel_steer_position,  // size() function pointer
    ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__get_const_function__SwerveControllerStatus__wheel_steer_position,  // get_const(index) function pointer
    ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__get_function__SwerveControllerStatus__wheel_steer_position,  // get(index) function pointer
    ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__fetch_function__SwerveControllerStatus__wheel_steer_position,  // fetch(index, &value) function pointer
    ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__assign_function__SwerveControllerStatus__wheel_steer_position,  // assign(index, value) function pointer
    ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__resize_function__SwerveControllerStatus__wheel_steer_position  // resize(index) function pointer
  },
  {
    "wheel_icr",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_controllers_interfaces__msg__SwerveControllerStatus, wheel_icr),  // bytes offset in struct
    NULL,  // default value
    ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__size_function__SwerveControllerStatus__wheel_icr,  // size() function pointer
    ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__get_const_function__SwerveControllerStatus__wheel_icr,  // get_const(index) function pointer
    ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__get_function__SwerveControllerStatus__wheel_icr,  // get(index) function pointer
    ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__fetch_function__SwerveControllerStatus__wheel_icr,  // fetch(index, &value) function pointer
    ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__assign_function__SwerveControllerStatus__wheel_icr,  // assign(index, value) function pointer
    ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__resize_function__SwerveControllerStatus__wheel_icr  // resize(index) function pointer
  },
  {
    "wheel_steering_angle_cmd",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_controllers_interfaces__msg__SwerveControllerStatus, wheel_steering_angle_cmd),  // bytes offset in struct
    NULL,  // default value
    ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__size_function__SwerveControllerStatus__wheel_steering_angle_cmd,  // size() function pointer
    ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__get_const_function__SwerveControllerStatus__wheel_steering_angle_cmd,  // get_const(index) function pointer
    ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__get_function__SwerveControllerStatus__wheel_steering_angle_cmd,  // get(index) function pointer
    ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__fetch_function__SwerveControllerStatus__wheel_steering_angle_cmd,  // fetch(index, &value) function pointer
    ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__assign_function__SwerveControllerStatus__wheel_steering_angle_cmd,  // assign(index, value) function pointer
    ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__resize_function__SwerveControllerStatus__wheel_steering_angle_cmd  // resize(index) function pointer
  },
  {
    "wheel_drive_velocity_cmd",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_controllers_interfaces__msg__SwerveControllerStatus, wheel_drive_velocity_cmd),  // bytes offset in struct
    NULL,  // default value
    ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__size_function__SwerveControllerStatus__wheel_drive_velocity_cmd,  // size() function pointer
    ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__get_const_function__SwerveControllerStatus__wheel_drive_velocity_cmd,  // get_const(index) function pointer
    ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__get_function__SwerveControllerStatus__wheel_drive_velocity_cmd,  // get(index) function pointer
    ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__fetch_function__SwerveControllerStatus__wheel_drive_velocity_cmd,  // fetch(index, &value) function pointer
    ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__assign_function__SwerveControllerStatus__wheel_drive_velocity_cmd,  // assign(index, value) function pointer
    ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__resize_function__SwerveControllerStatus__wheel_drive_velocity_cmd  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__SwerveControllerStatus_message_members = {
  "ros2_controllers_interfaces__msg",  // message namespace
  "SwerveControllerStatus",  // message name
  7,  // number of fields
  sizeof(ros2_controllers_interfaces__msg__SwerveControllerStatus),
  false,  // has_any_key_member_
  ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__SwerveControllerStatus_message_member_array,  // message members
  ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__SwerveControllerStatus_init_function,  // function to initialize message memory (memory has to be allocated)
  ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__SwerveControllerStatus_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__SwerveControllerStatus_message_type_support_handle = {
  0,
  &ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__SwerveControllerStatus_message_members,
  get_message_typesupport_handle_function,
  &ros2_controllers_interfaces__msg__SwerveControllerStatus__get_type_hash,
  &ros2_controllers_interfaces__msg__SwerveControllerStatus__get_type_description,
  &ros2_controllers_interfaces__msg__SwerveControllerStatus__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ros2_controllers_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros2_controllers_interfaces, msg, SwerveControllerStatus)() {
  ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__SwerveControllerStatus_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__SwerveControllerStatus_message_type_support_handle.typesupport_identifier) {
    ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__SwerveControllerStatus_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ros2_controllers_interfaces__msg__SwerveControllerStatus__rosidl_typesupport_introspection_c__SwerveControllerStatus_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
