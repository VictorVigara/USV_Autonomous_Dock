// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros_ign_interfaces:msg/ParamVec.idl
// generated code does not contain a copyright notice

#ifndef ROS_IGN_INTERFACES__MSG__DETAIL__PARAM_VEC__STRUCT_H_
#define ROS_IGN_INTERFACES__MSG__DETAIL__PARAM_VEC__STRUCT_H_

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
// Member 'params'
#include "rcl_interfaces/msg/detail/parameter__struct.h"

// Struct defined in msg/ParamVec in the package ros_ign_interfaces.
typedef struct ros_ign_interfaces__msg__ParamVec
{
  std_msgs__msg__Header header;
  rcl_interfaces__msg__Parameter__Sequence params;
} ros_ign_interfaces__msg__ParamVec;

// Struct for a sequence of ros_ign_interfaces__msg__ParamVec.
typedef struct ros_ign_interfaces__msg__ParamVec__Sequence
{
  ros_ign_interfaces__msg__ParamVec * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros_ign_interfaces__msg__ParamVec__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS_IGN_INTERFACES__MSG__DETAIL__PARAM_VEC__STRUCT_H_
