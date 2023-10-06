// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros_ign_interfaces:msg/EntityFactory.idl
// generated code does not contain a copyright notice

#ifndef ROS_IGN_INTERFACES__MSG__DETAIL__ENTITY_FACTORY__STRUCT_H_
#define ROS_IGN_INTERFACES__MSG__DETAIL__ENTITY_FACTORY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'name'
// Member 'sdf'
// Member 'sdf_filename'
// Member 'clone_name'
// Member 'relative_to'
#include "rosidl_runtime_c/string.h"
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.h"

// Struct defined in msg/EntityFactory in the package ros_ign_interfaces.
typedef struct ros_ign_interfaces__msg__EntityFactory
{
  rosidl_runtime_c__String name;
  bool allow_renaming;
  rosidl_runtime_c__String sdf;
  rosidl_runtime_c__String sdf_filename;
  rosidl_runtime_c__String clone_name;
  geometry_msgs__msg__Pose pose;
  rosidl_runtime_c__String relative_to;
} ros_ign_interfaces__msg__EntityFactory;

// Struct for a sequence of ros_ign_interfaces__msg__EntityFactory.
typedef struct ros_ign_interfaces__msg__EntityFactory__Sequence
{
  ros_ign_interfaces__msg__EntityFactory * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros_ign_interfaces__msg__EntityFactory__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS_IGN_INTERFACES__MSG__DETAIL__ENTITY_FACTORY__STRUCT_H_
