// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros_ign_interfaces:msg/Light.idl
// generated code does not contain a copyright notice

#ifndef ROS_IGN_INTERFACES__MSG__DETAIL__LIGHT__STRUCT_H_
#define ROS_IGN_INTERFACES__MSG__DETAIL__LIGHT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'POINT'.
enum
{
  ros_ign_interfaces__msg__Light__POINT = 0
};

/// Constant 'SPOT'.
enum
{
  ros_ign_interfaces__msg__Light__SPOT = 1
};

/// Constant 'DIRECTIONAL'.
enum
{
  ros_ign_interfaces__msg__Light__DIRECTIONAL = 2
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'name'
#include "rosidl_runtime_c/string.h"
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.h"
// Member 'diffuse'
// Member 'specular'
#include "std_msgs/msg/detail/color_rgba__struct.h"
// Member 'direction'
#include "geometry_msgs/msg/detail/vector3__struct.h"

// Struct defined in msg/Light in the package ros_ign_interfaces.
typedef struct ros_ign_interfaces__msg__Light
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__String name;
  uint8_t type;
  geometry_msgs__msg__Pose pose;
  std_msgs__msg__ColorRGBA diffuse;
  std_msgs__msg__ColorRGBA specular;
  float attenuation_constant;
  float attenuation_linear;
  float attenuation_quadratic;
  geometry_msgs__msg__Vector3 direction;
  float range;
  bool cast_shadows;
  float spot_inner_angle;
  float spot_outer_angle;
  float spot_falloff;
  uint32_t id;
  uint32_t parent_id;
  float intensity;
} ros_ign_interfaces__msg__Light;

// Struct for a sequence of ros_ign_interfaces__msg__Light.
typedef struct ros_ign_interfaces__msg__Light__Sequence
{
  ros_ign_interfaces__msg__Light * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros_ign_interfaces__msg__Light__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS_IGN_INTERFACES__MSG__DETAIL__LIGHT__STRUCT_H_
