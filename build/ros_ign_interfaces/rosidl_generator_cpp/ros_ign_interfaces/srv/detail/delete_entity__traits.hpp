// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ros_ign_interfaces:srv/DeleteEntity.idl
// generated code does not contain a copyright notice

#ifndef ROS_IGN_INTERFACES__SRV__DETAIL__DELETE_ENTITY__TRAITS_HPP_
#define ROS_IGN_INTERFACES__SRV__DETAIL__DELETE_ENTITY__TRAITS_HPP_

#include "ros_ign_interfaces/srv/detail/delete_entity__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'entity'
#include "ros_ign_interfaces/msg/detail/entity__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<ros_ign_interfaces::srv::DeleteEntity_Request>()
{
  return "ros_ign_interfaces::srv::DeleteEntity_Request";
}

template<>
inline const char * name<ros_ign_interfaces::srv::DeleteEntity_Request>()
{
  return "ros_ign_interfaces/srv/DeleteEntity_Request";
}

template<>
struct has_fixed_size<ros_ign_interfaces::srv::DeleteEntity_Request>
  : std::integral_constant<bool, has_fixed_size<ros_ign_interfaces::msg::Entity>::value> {};

template<>
struct has_bounded_size<ros_ign_interfaces::srv::DeleteEntity_Request>
  : std::integral_constant<bool, has_bounded_size<ros_ign_interfaces::msg::Entity>::value> {};

template<>
struct is_message<ros_ign_interfaces::srv::DeleteEntity_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<ros_ign_interfaces::srv::DeleteEntity_Response>()
{
  return "ros_ign_interfaces::srv::DeleteEntity_Response";
}

template<>
inline const char * name<ros_ign_interfaces::srv::DeleteEntity_Response>()
{
  return "ros_ign_interfaces/srv/DeleteEntity_Response";
}

template<>
struct has_fixed_size<ros_ign_interfaces::srv::DeleteEntity_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<ros_ign_interfaces::srv::DeleteEntity_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<ros_ign_interfaces::srv::DeleteEntity_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<ros_ign_interfaces::srv::DeleteEntity>()
{
  return "ros_ign_interfaces::srv::DeleteEntity";
}

template<>
inline const char * name<ros_ign_interfaces::srv::DeleteEntity>()
{
  return "ros_ign_interfaces/srv/DeleteEntity";
}

template<>
struct has_fixed_size<ros_ign_interfaces::srv::DeleteEntity>
  : std::integral_constant<
    bool,
    has_fixed_size<ros_ign_interfaces::srv::DeleteEntity_Request>::value &&
    has_fixed_size<ros_ign_interfaces::srv::DeleteEntity_Response>::value
  >
{
};

template<>
struct has_bounded_size<ros_ign_interfaces::srv::DeleteEntity>
  : std::integral_constant<
    bool,
    has_bounded_size<ros_ign_interfaces::srv::DeleteEntity_Request>::value &&
    has_bounded_size<ros_ign_interfaces::srv::DeleteEntity_Response>::value
  >
{
};

template<>
struct is_service<ros_ign_interfaces::srv::DeleteEntity>
  : std::true_type
{
};

template<>
struct is_service_request<ros_ign_interfaces::srv::DeleteEntity_Request>
  : std::true_type
{
};

template<>
struct is_service_response<ros_ign_interfaces::srv::DeleteEntity_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ROS_IGN_INTERFACES__SRV__DETAIL__DELETE_ENTITY__TRAITS_HPP_
