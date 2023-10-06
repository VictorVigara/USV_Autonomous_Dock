// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ros_ign_interfaces:msg/WorldControl.idl
// generated code does not contain a copyright notice

#ifndef ROS_IGN_INTERFACES__MSG__DETAIL__WORLD_CONTROL__TRAITS_HPP_
#define ROS_IGN_INTERFACES__MSG__DETAIL__WORLD_CONTROL__TRAITS_HPP_

#include "ros_ign_interfaces/msg/detail/world_control__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'reset'
#include "ros_ign_interfaces/msg/detail/world_reset__traits.hpp"
// Member 'run_to_sim_time'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<ros_ign_interfaces::msg::WorldControl>()
{
  return "ros_ign_interfaces::msg::WorldControl";
}

template<>
inline const char * name<ros_ign_interfaces::msg::WorldControl>()
{
  return "ros_ign_interfaces/msg/WorldControl";
}

template<>
struct has_fixed_size<ros_ign_interfaces::msg::WorldControl>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value && has_fixed_size<ros_ign_interfaces::msg::WorldReset>::value> {};

template<>
struct has_bounded_size<ros_ign_interfaces::msg::WorldControl>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value && has_bounded_size<ros_ign_interfaces::msg::WorldReset>::value> {};

template<>
struct is_message<ros_ign_interfaces::msg::WorldControl>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROS_IGN_INTERFACES__MSG__DETAIL__WORLD_CONTROL__TRAITS_HPP_
