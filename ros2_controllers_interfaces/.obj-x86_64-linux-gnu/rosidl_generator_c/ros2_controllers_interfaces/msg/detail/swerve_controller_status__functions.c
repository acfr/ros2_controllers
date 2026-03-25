// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ros2_controllers_interfaces:msg/SwerveControllerStatus.idl
// generated code does not contain a copyright notice
#include "ros2_controllers_interfaces/msg/detail/swerve_controller_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `wheel_drive_position`
// Member `wheel_drive_velocity`
// Member `wheel_steer_position`
// Member `wheel_icr`
// Member `wheel_steering_angle_cmd`
// Member `wheel_drive_velocity_cmd`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
ros2_controllers_interfaces__msg__SwerveControllerStatus__init(ros2_controllers_interfaces__msg__SwerveControllerStatus * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    ros2_controllers_interfaces__msg__SwerveControllerStatus__fini(msg);
    return false;
  }
  // wheel_drive_position
  if (!rosidl_runtime_c__double__Sequence__init(&msg->wheel_drive_position, 0)) {
    ros2_controllers_interfaces__msg__SwerveControllerStatus__fini(msg);
    return false;
  }
  // wheel_drive_velocity
  if (!rosidl_runtime_c__double__Sequence__init(&msg->wheel_drive_velocity, 0)) {
    ros2_controllers_interfaces__msg__SwerveControllerStatus__fini(msg);
    return false;
  }
  // wheel_steer_position
  if (!rosidl_runtime_c__double__Sequence__init(&msg->wheel_steer_position, 0)) {
    ros2_controllers_interfaces__msg__SwerveControllerStatus__fini(msg);
    return false;
  }
  // wheel_icr
  if (!rosidl_runtime_c__double__Sequence__init(&msg->wheel_icr, 0)) {
    ros2_controllers_interfaces__msg__SwerveControllerStatus__fini(msg);
    return false;
  }
  // wheel_steering_angle_cmd
  if (!rosidl_runtime_c__double__Sequence__init(&msg->wheel_steering_angle_cmd, 0)) {
    ros2_controllers_interfaces__msg__SwerveControllerStatus__fini(msg);
    return false;
  }
  // wheel_drive_velocity_cmd
  if (!rosidl_runtime_c__double__Sequence__init(&msg->wheel_drive_velocity_cmd, 0)) {
    ros2_controllers_interfaces__msg__SwerveControllerStatus__fini(msg);
    return false;
  }
  return true;
}

void
ros2_controllers_interfaces__msg__SwerveControllerStatus__fini(ros2_controllers_interfaces__msg__SwerveControllerStatus * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // wheel_drive_position
  rosidl_runtime_c__double__Sequence__fini(&msg->wheel_drive_position);
  // wheel_drive_velocity
  rosidl_runtime_c__double__Sequence__fini(&msg->wheel_drive_velocity);
  // wheel_steer_position
  rosidl_runtime_c__double__Sequence__fini(&msg->wheel_steer_position);
  // wheel_icr
  rosidl_runtime_c__double__Sequence__fini(&msg->wheel_icr);
  // wheel_steering_angle_cmd
  rosidl_runtime_c__double__Sequence__fini(&msg->wheel_steering_angle_cmd);
  // wheel_drive_velocity_cmd
  rosidl_runtime_c__double__Sequence__fini(&msg->wheel_drive_velocity_cmd);
}

bool
ros2_controllers_interfaces__msg__SwerveControllerStatus__are_equal(const ros2_controllers_interfaces__msg__SwerveControllerStatus * lhs, const ros2_controllers_interfaces__msg__SwerveControllerStatus * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // wheel_drive_position
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->wheel_drive_position), &(rhs->wheel_drive_position)))
  {
    return false;
  }
  // wheel_drive_velocity
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->wheel_drive_velocity), &(rhs->wheel_drive_velocity)))
  {
    return false;
  }
  // wheel_steer_position
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->wheel_steer_position), &(rhs->wheel_steer_position)))
  {
    return false;
  }
  // wheel_icr
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->wheel_icr), &(rhs->wheel_icr)))
  {
    return false;
  }
  // wheel_steering_angle_cmd
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->wheel_steering_angle_cmd), &(rhs->wheel_steering_angle_cmd)))
  {
    return false;
  }
  // wheel_drive_velocity_cmd
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->wheel_drive_velocity_cmd), &(rhs->wheel_drive_velocity_cmd)))
  {
    return false;
  }
  return true;
}

bool
ros2_controllers_interfaces__msg__SwerveControllerStatus__copy(
  const ros2_controllers_interfaces__msg__SwerveControllerStatus * input,
  ros2_controllers_interfaces__msg__SwerveControllerStatus * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // wheel_drive_position
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->wheel_drive_position), &(output->wheel_drive_position)))
  {
    return false;
  }
  // wheel_drive_velocity
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->wheel_drive_velocity), &(output->wheel_drive_velocity)))
  {
    return false;
  }
  // wheel_steer_position
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->wheel_steer_position), &(output->wheel_steer_position)))
  {
    return false;
  }
  // wheel_icr
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->wheel_icr), &(output->wheel_icr)))
  {
    return false;
  }
  // wheel_steering_angle_cmd
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->wheel_steering_angle_cmd), &(output->wheel_steering_angle_cmd)))
  {
    return false;
  }
  // wheel_drive_velocity_cmd
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->wheel_drive_velocity_cmd), &(output->wheel_drive_velocity_cmd)))
  {
    return false;
  }
  return true;
}

ros2_controllers_interfaces__msg__SwerveControllerStatus *
ros2_controllers_interfaces__msg__SwerveControllerStatus__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2_controllers_interfaces__msg__SwerveControllerStatus * msg = (ros2_controllers_interfaces__msg__SwerveControllerStatus *)allocator.allocate(sizeof(ros2_controllers_interfaces__msg__SwerveControllerStatus), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ros2_controllers_interfaces__msg__SwerveControllerStatus));
  bool success = ros2_controllers_interfaces__msg__SwerveControllerStatus__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ros2_controllers_interfaces__msg__SwerveControllerStatus__destroy(ros2_controllers_interfaces__msg__SwerveControllerStatus * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ros2_controllers_interfaces__msg__SwerveControllerStatus__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ros2_controllers_interfaces__msg__SwerveControllerStatus__Sequence__init(ros2_controllers_interfaces__msg__SwerveControllerStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2_controllers_interfaces__msg__SwerveControllerStatus * data = NULL;

  if (size) {
    data = (ros2_controllers_interfaces__msg__SwerveControllerStatus *)allocator.zero_allocate(size, sizeof(ros2_controllers_interfaces__msg__SwerveControllerStatus), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ros2_controllers_interfaces__msg__SwerveControllerStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ros2_controllers_interfaces__msg__SwerveControllerStatus__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
ros2_controllers_interfaces__msg__SwerveControllerStatus__Sequence__fini(ros2_controllers_interfaces__msg__SwerveControllerStatus__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      ros2_controllers_interfaces__msg__SwerveControllerStatus__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

ros2_controllers_interfaces__msg__SwerveControllerStatus__Sequence *
ros2_controllers_interfaces__msg__SwerveControllerStatus__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2_controllers_interfaces__msg__SwerveControllerStatus__Sequence * array = (ros2_controllers_interfaces__msg__SwerveControllerStatus__Sequence *)allocator.allocate(sizeof(ros2_controllers_interfaces__msg__SwerveControllerStatus__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ros2_controllers_interfaces__msg__SwerveControllerStatus__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ros2_controllers_interfaces__msg__SwerveControllerStatus__Sequence__destroy(ros2_controllers_interfaces__msg__SwerveControllerStatus__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ros2_controllers_interfaces__msg__SwerveControllerStatus__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ros2_controllers_interfaces__msg__SwerveControllerStatus__Sequence__are_equal(const ros2_controllers_interfaces__msg__SwerveControllerStatus__Sequence * lhs, const ros2_controllers_interfaces__msg__SwerveControllerStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ros2_controllers_interfaces__msg__SwerveControllerStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ros2_controllers_interfaces__msg__SwerveControllerStatus__Sequence__copy(
  const ros2_controllers_interfaces__msg__SwerveControllerStatus__Sequence * input,
  ros2_controllers_interfaces__msg__SwerveControllerStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ros2_controllers_interfaces__msg__SwerveControllerStatus);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ros2_controllers_interfaces__msg__SwerveControllerStatus * data =
      (ros2_controllers_interfaces__msg__SwerveControllerStatus *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ros2_controllers_interfaces__msg__SwerveControllerStatus__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ros2_controllers_interfaces__msg__SwerveControllerStatus__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ros2_controllers_interfaces__msg__SwerveControllerStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
