// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from ros_ign_interfaces:msg/Dataframe.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "ros_ign_interfaces/msg/detail/dataframe__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace ros_ign_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void Dataframe_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) ros_ign_interfaces::msg::Dataframe(_init);
}

void Dataframe_fini_function(void * message_memory)
{
  auto typed_message = static_cast<ros_ign_interfaces::msg::Dataframe *>(message_memory);
  typed_message->~Dataframe();
}

size_t size_function__Dataframe__data(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<uint8_t> *>(untyped_member);
  return member->size();
}

const void * get_const_function__Dataframe__data(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<uint8_t> *>(untyped_member);
  return &member[index];
}

void * get_function__Dataframe__data(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<uint8_t> *>(untyped_member);
  return &member[index];
}

void resize_function__Dataframe__data(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<uint8_t> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember Dataframe_message_member_array[5] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros_ign_interfaces::msg::Dataframe, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "src_address",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros_ign_interfaces::msg::Dataframe, src_address),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "dst_address",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros_ign_interfaces::msg::Dataframe, dst_address),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "data",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros_ign_interfaces::msg::Dataframe, data),  // bytes offset in struct
    nullptr,  // default value
    size_function__Dataframe__data,  // size() function pointer
    get_const_function__Dataframe__data,  // get_const(index) function pointer
    get_function__Dataframe__data,  // get(index) function pointer
    resize_function__Dataframe__data  // resize(index) function pointer
  },
  {
    "rssi",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros_ign_interfaces::msg::Dataframe, rssi),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers Dataframe_message_members = {
  "ros_ign_interfaces::msg",  // message namespace
  "Dataframe",  // message name
  5,  // number of fields
  sizeof(ros_ign_interfaces::msg::Dataframe),
  Dataframe_message_member_array,  // message members
  Dataframe_init_function,  // function to initialize message memory (memory has to be allocated)
  Dataframe_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t Dataframe_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &Dataframe_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace ros_ign_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<ros_ign_interfaces::msg::Dataframe>()
{
  return &::ros_ign_interfaces::msg::rosidl_typesupport_introspection_cpp::Dataframe_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, ros_ign_interfaces, msg, Dataframe)() {
  return &::ros_ign_interfaces::msg::rosidl_typesupport_introspection_cpp::Dataframe_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
