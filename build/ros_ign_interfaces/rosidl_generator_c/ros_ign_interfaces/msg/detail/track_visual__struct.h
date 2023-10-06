// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros_ign_interfaces:msg/TrackVisual.idl
// generated code does not contain a copyright notice

#ifndef ROS_IGN_INTERFACES__MSG__DETAIL__TRACK_VISUAL__STRUCT_H_
#define ROS_IGN_INTERFACES__MSG__DETAIL__TRACK_VISUAL__STRUCT_H_

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
// Member 'name'
#include "rosidl_runtime_c/string.h"
// Member 'xyz'
#include "geometry_msgs/msg/detail/vector3__struct.h"

// Struct defined in msg/TrackVisual in the package ros_ign_interfaces.
typedef struct ros_ign_interfaces__msg__TrackVisual
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__String name;
  uint32_t id;
  bool inherit_orientation;
  double min_dist;
  double max_dist;
  bool is_static;
  bool use_model_frame;
  geometry_msgs__msg__Vector3 xyz;
  bool inherit_yaw;
} ros_ign_interfaces__msg__TrackVisual;

// Struct for a sequence of ros_ign_interfaces__msg__TrackVisual.
typedef struct ros_ign_interfaces__msg__TrackVisual__Sequence
{
  ros_ign_interfaces__msg__TrackVisual * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros_ign_interfaces__msg__TrackVisual__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS_IGN_INTERFACES__MSG__DETAIL__TRACK_VISUAL__STRUCT_H_
