// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from ros_ign_interfaces:msg/ParamVec.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "ros_ign_interfaces/msg/detail/param_vec__rosidl_typesupport_introspection_c.h"
#include "ros_ign_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "ros_ign_interfaces/msg/detail/param_vec__functions.h"
#include "ros_ign_interfaces/msg/detail/param_vec__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `params`
#include "rcl_interfaces/msg/parameter.h"
// Member `params`
#include "rcl_interfaces/msg/detail/parameter__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void ParamVec__rosidl_typesupport_introspection_c__ParamVec_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  ros_ign_interfaces__msg__ParamVec__init(message_memory);
}

void ParamVec__rosidl_typesupport_introspection_c__ParamVec_fini_function(void * message_memory)
{
  ros_ign_interfaces__msg__ParamVec__fini(message_memory);
}

size_t ParamVec__rosidl_typesupport_introspection_c__size_function__Parameter__params(
  const void * untyped_member)
{
  const rcl_interfaces__msg__Parameter__Sequence * member =
    (const rcl_interfaces__msg__Parameter__Sequence *)(untyped_member);
  return member->size;
}

const void * ParamVec__rosidl_typesupport_introspection_c__get_const_function__Parameter__params(
  const void * untyped_member, size_t index)
{
  const rcl_interfaces__msg__Parameter__Sequence * member =
    (const rcl_interfaces__msg__Parameter__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ParamVec__rosidl_typesupport_introspection_c__get_function__Parameter__params(
  void * untyped_member, size_t index)
{
  rcl_interfaces__msg__Parameter__Sequence * member =
    (rcl_interfaces__msg__Parameter__Sequence *)(untyped_member);
  return &member->data[index];
}

bool ParamVec__rosidl_typesupport_introspection_c__resize_function__Parameter__params(
  void * untyped_member, size_t size)
{
  rcl_interfaces__msg__Parameter__Sequence * member =
    (rcl_interfaces__msg__Parameter__Sequence *)(untyped_member);
  rcl_interfaces__msg__Parameter__Sequence__fini(member);
  return rcl_interfaces__msg__Parameter__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember ParamVec__rosidl_typesupport_introspection_c__ParamVec_message_member_array[2] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros_ign_interfaces__msg__ParamVec, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "params",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros_ign_interfaces__msg__ParamVec, params),  // bytes offset in struct
    NULL,  // default value
    ParamVec__rosidl_typesupport_introspection_c__size_function__Parameter__params,  // size() function pointer
    ParamVec__rosidl_typesupport_introspection_c__get_const_function__Parameter__params,  // get_const(index) function pointer
    ParamVec__rosidl_typesupport_introspection_c__get_function__Parameter__params,  // get(index) function pointer
    ParamVec__rosidl_typesupport_introspection_c__resize_function__Parameter__params  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ParamVec__rosidl_typesupport_introspection_c__ParamVec_message_members = {
  "ros_ign_interfaces__msg",  // message namespace
  "ParamVec",  // message name
  2,  // number of fields
  sizeof(ros_ign_interfaces__msg__ParamVec),
  ParamVec__rosidl_typesupport_introspection_c__ParamVec_message_member_array,  // message members
  ParamVec__rosidl_typesupport_introspection_c__ParamVec_init_function,  // function to initialize message memory (memory has to be allocated)
  ParamVec__rosidl_typesupport_introspection_c__ParamVec_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ParamVec__rosidl_typesupport_introspection_c__ParamVec_message_type_support_handle = {
  0,
  &ParamVec__rosidl_typesupport_introspection_c__ParamVec_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ros_ign_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros_ign_interfaces, msg, ParamVec)() {
  ParamVec__rosidl_typesupport_introspection_c__ParamVec_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  ParamVec__rosidl_typesupport_introspection_c__ParamVec_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rcl_interfaces, msg, Parameter)();
  if (!ParamVec__rosidl_typesupport_introspection_c__ParamVec_message_type_support_handle.typesupport_identifier) {
    ParamVec__rosidl_typesupport_introspection_c__ParamVec_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ParamVec__rosidl_typesupport_introspection_c__ParamVec_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
