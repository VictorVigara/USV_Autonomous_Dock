// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ros_ign_interfaces:msg/WorldReset.idl
// generated code does not contain a copyright notice

#ifndef ROS_IGN_INTERFACES__MSG__DETAIL__WORLD_RESET__TRAITS_HPP_
#define ROS_IGN_INTERFACES__MSG__DETAIL__WORLD_RESET__TRAITS_HPP_

#include "ros_ign_interfaces/msg/detail/world_reset__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<ros_ign_interfaces::msg::WorldReset>()
{
  return "ros_ign_interfaces::msg::WorldReset";
}

template<>
inline const char * name<ros_ign_interfaces::msg::WorldReset>()
{
  return "ros_ign_interfaces/msg/WorldReset";
}

template<>
struct has_fixed_size<ros_ign_interfaces::msg::WorldReset>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<ros_ign_interfaces::msg::WorldReset>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<ros_ign_interfaces::msg::WorldReset>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROS_IGN_INTERFACES__MSG__DETAIL__WORLD_RESET__TRAITS_HPP_
