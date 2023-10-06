// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros_ign_interfaces:msg/ParamVec.idl
// generated code does not contain a copyright notice

#ifndef ROS_IGN_INTERFACES__MSG__DETAIL__PARAM_VEC__BUILDER_HPP_
#define ROS_IGN_INTERFACES__MSG__DETAIL__PARAM_VEC__BUILDER_HPP_

#include "ros_ign_interfaces/msg/detail/param_vec__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace ros_ign_interfaces
{

namespace msg
{

namespace builder
{

class Init_ParamVec_params
{
public:
  explicit Init_ParamVec_params(::ros_ign_interfaces::msg::ParamVec & msg)
  : msg_(msg)
  {}
  ::ros_ign_interfaces::msg::ParamVec params(::ros_ign_interfaces::msg::ParamVec::_params_type arg)
  {
    msg_.params = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros_ign_interfaces::msg::ParamVec msg_;
};

class Init_ParamVec_header
{
public:
  Init_ParamVec_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ParamVec_params header(::ros_ign_interfaces::msg::ParamVec::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ParamVec_params(msg_);
  }

private:
  ::ros_ign_interfaces::msg::ParamVec msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros_ign_interfaces::msg::ParamVec>()
{
  return ros_ign_interfaces::msg::builder::Init_ParamVec_header();
}

}  // namespace ros_ign_interfaces

#endif  // ROS_IGN_INTERFACES__MSG__DETAIL__PARAM_VEC__BUILDER_HPP_
