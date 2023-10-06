// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from ros_ign_interfaces:msg/StringVec.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "ros_ign_interfaces/msg/detail/string_vec__rosidl_typesupport_introspection_c.h"
#include "ros_ign_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "ros_ign_interfaces/msg/detail/string_vec__functions.h"
#include "ros_ign_interfaces/msg/detail/string_vec__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `data`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void StringVec__rosidl_typesupport_introspection_c__StringVec_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  ros_ign_interfaces__msg__StringVec__init(message_memory);
}

void StringVec__rosidl_typesupport_introspection_c__StringVec_fini_function(void * message_memory)
{
  ros_ign_interfaces__msg__StringVec__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember StringVec__rosidl_typesupport_introspection_c__StringVec_message_member_array[2] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros_ign_interfaces__msg__StringVec, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "data",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros_ign_interfaces__msg__StringVec, data),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers StringVec__rosidl_typesupport_introspection_c__StringVec_message_members = {
  "ros_ign_interfaces__msg",  // message namespace
  "StringVec",  // message name
  2,  // number of fields
  sizeof(ros_ign_interfaces__msg__StringVec),
  StringVec__rosidl_typesupport_introspection_c__StringVec_message_member_array,  // message members
  StringVec__rosidl_typesupport_introspection_c__StringVec_init_function,  // function to initialize message memory (memory has to be allocated)
  StringVec__rosidl_typesupport_introspection_c__StringVec_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t StringVec__rosidl_typesupport_introspection_c__StringVec_message_type_support_handle = {
  0,
  &StringVec__rosidl_typesupport_introspection_c__StringVec_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ros_ign_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros_ign_interfaces, msg, StringVec)() {
  StringVec__rosidl_typesupport_introspection_c__StringVec_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!StringVec__rosidl_typesupport_introspection_c__StringVec_message_type_support_handle.typesupport_identifier) {
    StringVec__rosidl_typesupport_introspection_c__StringVec_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &StringVec__rosidl_typesupport_introspection_c__StringVec_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
