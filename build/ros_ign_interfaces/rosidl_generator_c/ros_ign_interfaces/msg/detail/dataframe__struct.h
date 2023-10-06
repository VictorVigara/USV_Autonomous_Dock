// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros_ign_interfaces:msg/Dataframe.idl
// generated code does not contain a copyright notice

#ifndef ROS_IGN_INTERFACES__MSG__DETAIL__DATAFRAME__STRUCT_H_
#define ROS_IGN_INTERFACES__MSG__DETAIL__DATAFRAME__STRUCT_H_

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
// Member 'src_address'
// Member 'dst_address'
#include "rosidl_runtime_c/string.h"
// Member 'data'
#include "rosidl_runtime_c/primitives_sequence.h"

// Struct defined in msg/Dataframe in the package ros_ign_interfaces.
typedef struct ros_ign_interfaces__msg__Dataframe
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__String src_address;
  rosidl_runtime_c__String dst_address;
  rosidl_runtime_c__uint8__Sequence data;
  double rssi;
} ros_ign_interfaces__msg__Dataframe;

// Struct for a sequence of ros_ign_interfaces__msg__Dataframe.
typedef struct ros_ign_interfaces__msg__Dataframe__Sequence
{
  ros_ign_interfaces__msg__Dataframe * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros_ign_interfaces__msg__Dataframe__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS_IGN_INTERFACES__MSG__DETAIL__DATAFRAME__STRUCT_H_
