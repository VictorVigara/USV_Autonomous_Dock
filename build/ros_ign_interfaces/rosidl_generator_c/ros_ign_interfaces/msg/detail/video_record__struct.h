// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros_ign_interfaces:msg/VideoRecord.idl
// generated code does not contain a copyright notice

#ifndef ROS_IGN_INTERFACES__MSG__DETAIL__VIDEO_RECORD__STRUCT_H_
#define ROS_IGN_INTERFACES__MSG__DETAIL__VIDEO_RECORD__STRUCT_H_

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
// Member 'format'
// Member 'save_filename'
#include "rosidl_runtime_c/string.h"

// Struct defined in msg/VideoRecord in the package ros_ign_interfaces.
typedef struct ros_ign_interfaces__msg__VideoRecord
{
  std_msgs__msg__Header header;
  bool start;
  bool stop;
  rosidl_runtime_c__String format;
  rosidl_runtime_c__String save_filename;
} ros_ign_interfaces__msg__VideoRecord;

// Struct for a sequence of ros_ign_interfaces__msg__VideoRecord.
typedef struct ros_ign_interfaces__msg__VideoRecord__Sequence
{
  ros_ign_interfaces__msg__VideoRecord * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros_ign_interfaces__msg__VideoRecord__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS_IGN_INTERFACES__MSG__DETAIL__VIDEO_RECORD__STRUCT_H_
