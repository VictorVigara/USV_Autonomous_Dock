// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ros_ign_interfaces:msg/EntityFactory.idl
// generated code does not contain a copyright notice

#ifndef ROS_IGN_INTERFACES__MSG__DETAIL__ENTITY_FACTORY__TRAITS_HPP_
#define ROS_IGN_INTERFACES__MSG__DETAIL__ENTITY_FACTORY__TRAITS_HPP_

#include "ros_ign_interfaces/msg/detail/entity_factory__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<ros_ign_interfaces::msg::EntityFactory>()
{
  return "ros_ign_interfaces::msg::EntityFactory";
}

template<>
inline const char * name<ros_ign_interfaces::msg::EntityFactory>()
{
  return "ros_ign_interfaces/msg/EntityFactory";
}

template<>
struct has_fixed_size<ros_ign_interfaces::msg::EntityFactory>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ros_ign_interfaces::msg::EntityFactory>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<ros_ign_interfaces::msg::EntityFactory>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROS_IGN_INTERFACES__MSG__DETAIL__ENTITY_FACTORY__TRAITS_HPP_
