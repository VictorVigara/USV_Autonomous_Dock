// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ros_ign_interfaces:msg/JointWrench.idl
// generated code does not contain a copyright notice

#ifndef ROS_IGN_INTERFACES__MSG__DETAIL__JOINT_WRENCH__TRAITS_HPP_
#define ROS_IGN_INTERFACES__MSG__DETAIL__JOINT_WRENCH__TRAITS_HPP_

#include "ros_ign_interfaces/msg/detail/joint_wrench__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'body_1_name'
// Member 'body_2_name'
#include "std_msgs/msg/detail/string__traits.hpp"
// Member 'body_1_id'
// Member 'body_2_id'
#include "std_msgs/msg/detail/u_int32__traits.hpp"
// Member 'body_1_wrench'
// Member 'body_2_wrench'
#include "geometry_msgs/msg/detail/wrench__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<ros_ign_interfaces::msg::JointWrench>()
{
  return "ros_ign_interfaces::msg::JointWrench";
}

template<>
inline const char * name<ros_ign_interfaces::msg::JointWrench>()
{
  return "ros_ign_interfaces/msg/JointWrench";
}

template<>
struct has_fixed_size<ros_ign_interfaces::msg::JointWrench>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Wrench>::value && has_fixed_size<std_msgs::msg::Header>::value && has_fixed_size<std_msgs::msg::String>::value && has_fixed_size<std_msgs::msg::UInt32>::value> {};

template<>
struct has_bounded_size<ros_ign_interfaces::msg::JointWrench>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Wrench>::value && has_bounded_size<std_msgs::msg::Header>::value && has_bounded_size<std_msgs::msg::String>::value && has_bounded_size<std_msgs::msg::UInt32>::value> {};

template<>
struct is_message<ros_ign_interfaces::msg::JointWrench>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROS_IGN_INTERFACES__MSG__DETAIL__JOINT_WRENCH__TRAITS_HPP_
