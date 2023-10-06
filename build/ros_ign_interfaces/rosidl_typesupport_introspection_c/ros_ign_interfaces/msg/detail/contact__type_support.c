// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from ros_ign_interfaces:msg/Contact.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "ros_ign_interfaces/msg/detail/contact__rosidl_typesupport_introspection_c.h"
#include "ros_ign_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "ros_ign_interfaces/msg/detail/contact__functions.h"
#include "ros_ign_interfaces/msg/detail/contact__struct.h"


// Include directives for member types
// Member `collision1`
// Member `collision2`
#include "ros_ign_interfaces/msg/entity.h"
// Member `collision1`
// Member `collision2`
#include "ros_ign_interfaces/msg/detail/entity__rosidl_typesupport_introspection_c.h"
// Member `positions`
// Member `normals`
#include "geometry_msgs/msg/vector3.h"
// Member `positions`
// Member `normals`
#include "geometry_msgs/msg/detail/vector3__rosidl_typesupport_introspection_c.h"
// Member `depths`
#include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `wrenches`
#include "ros_ign_interfaces/msg/joint_wrench.h"
// Member `wrenches`
#include "ros_ign_interfaces/msg/detail/joint_wrench__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void Contact__rosidl_typesupport_introspection_c__Contact_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  ros_ign_interfaces__msg__Contact__init(message_memory);
}

void Contact__rosidl_typesupport_introspection_c__Contact_fini_function(void * message_memory)
{
  ros_ign_interfaces__msg__Contact__fini(message_memory);
}

size_t Contact__rosidl_typesupport_introspection_c__size_function__Vector3__positions(
  const void * untyped_member)
{
  const geometry_msgs__msg__Vector3__Sequence * member =
    (const geometry_msgs__msg__Vector3__Sequence *)(untyped_member);
  return member->size;
}

const void * Contact__rosidl_typesupport_introspection_c__get_const_function__Vector3__positions(
  const void * untyped_member, size_t index)
{
  const geometry_msgs__msg__Vector3__Sequence * member =
    (const geometry_msgs__msg__Vector3__Sequence *)(untyped_member);
  return &member->data[index];
}

void * Contact__rosidl_typesupport_introspection_c__get_function__Vector3__positions(
  void * untyped_member, size_t index)
{
  geometry_msgs__msg__Vector3__Sequence * member =
    (geometry_msgs__msg__Vector3__Sequence *)(untyped_member);
  return &member->data[index];
}

bool Contact__rosidl_typesupport_introspection_c__resize_function__Vector3__positions(
  void * untyped_member, size_t size)
{
  geometry_msgs__msg__Vector3__Sequence * member =
    (geometry_msgs__msg__Vector3__Sequence *)(untyped_member);
  geometry_msgs__msg__Vector3__Sequence__fini(member);
  return geometry_msgs__msg__Vector3__Sequence__init(member, size);
}

size_t Contact__rosidl_typesupport_introspection_c__size_function__Vector3__normals(
  const void * untyped_member)
{
  const geometry_msgs__msg__Vector3__Sequence * member =
    (const geometry_msgs__msg__Vector3__Sequence *)(untyped_member);
  return member->size;
}

const void * Contact__rosidl_typesupport_introspection_c__get_const_function__Vector3__normals(
  const void * untyped_member, size_t index)
{
  const geometry_msgs__msg__Vector3__Sequence * member =
    (const geometry_msgs__msg__Vector3__Sequence *)(untyped_member);
  return &member->data[index];
}

void * Contact__rosidl_typesupport_introspection_c__get_function__Vector3__normals(
  void * untyped_member, size_t index)
{
  geometry_msgs__msg__Vector3__Sequence * member =
    (geometry_msgs__msg__Vector3__Sequence *)(untyped_member);
  return &member->data[index];
}

bool Contact__rosidl_typesupport_introspection_c__resize_function__Vector3__normals(
  void * untyped_member, size_t size)
{
  geometry_msgs__msg__Vector3__Sequence * member =
    (geometry_msgs__msg__Vector3__Sequence *)(untyped_member);
  geometry_msgs__msg__Vector3__Sequence__fini(member);
  return geometry_msgs__msg__Vector3__Sequence__init(member, size);
}

size_t Contact__rosidl_typesupport_introspection_c__size_function__JointWrench__wrenches(
  const void * untyped_member)
{
  const ros_ign_interfaces__msg__JointWrench__Sequence * member =
    (const ros_ign_interfaces__msg__JointWrench__Sequence *)(untyped_member);
  return member->size;
}

const void * Contact__rosidl_typesupport_introspection_c__get_const_function__JointWrench__wrenches(
  const void * untyped_member, size_t index)
{
  const ros_ign_interfaces__msg__JointWrench__Sequence * member =
    (const ros_ign_interfaces__msg__JointWrench__Sequence *)(untyped_member);
  return &member->data[index];
}

void * Contact__rosidl_typesupport_introspection_c__get_function__JointWrench__wrenches(
  void * untyped_member, size_t index)
{
  ros_ign_interfaces__msg__JointWrench__Sequence * member =
    (ros_ign_interfaces__msg__JointWrench__Sequence *)(untyped_member);
  return &member->data[index];
}

bool Contact__rosidl_typesupport_introspection_c__resize_function__JointWrench__wrenches(
  void * untyped_member, size_t size)
{
  ros_ign_interfaces__msg__JointWrench__Sequence * member =
    (ros_ign_interfaces__msg__JointWrench__Sequence *)(untyped_member);
  ros_ign_interfaces__msg__JointWrench__Sequence__fini(member);
  return ros_ign_interfaces__msg__JointWrench__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember Contact__rosidl_typesupport_introspection_c__Contact_message_member_array[6] = {
  {
    "collision1",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros_ign_interfaces__msg__Contact, collision1),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "collision2",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros_ign_interfaces__msg__Contact, collision2),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "positions",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros_ign_interfaces__msg__Contact, positions),  // bytes offset in struct
    NULL,  // default value
    Contact__rosidl_typesupport_introspection_c__size_function__Vector3__positions,  // size() function pointer
    Contact__rosidl_typesupport_introspection_c__get_const_function__Vector3__positions,  // get_const(index) function pointer
    Contact__rosidl_typesupport_introspection_c__get_function__Vector3__positions,  // get(index) function pointer
    Contact__rosidl_typesupport_introspection_c__resize_function__Vector3__positions  // resize(index) function pointer
  },
  {
    "normals",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros_ign_interfaces__msg__Contact, normals),  // bytes offset in struct
    NULL,  // default value
    Contact__rosidl_typesupport_introspection_c__size_function__Vector3__normals,  // size() function pointer
    Contact__rosidl_typesupport_introspection_c__get_const_function__Vector3__normals,  // get_const(index) function pointer
    Contact__rosidl_typesupport_introspection_c__get_function__Vector3__normals,  // get(index) function pointer
    Contact__rosidl_typesupport_introspection_c__resize_function__Vector3__normals  // resize(index) function pointer
  },
  {
    "depths",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros_ign_interfaces__msg__Contact, depths),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "wrenches",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros_ign_interfaces__msg__Contact, wrenches),  // bytes offset in struct
    NULL,  // default value
    Contact__rosidl_typesupport_introspection_c__size_function__JointWrench__wrenches,  // size() function pointer
    Contact__rosidl_typesupport_introspection_c__get_const_function__JointWrench__wrenches,  // get_const(index) function pointer
    Contact__rosidl_typesupport_introspection_c__get_function__JointWrench__wrenches,  // get(index) function pointer
    Contact__rosidl_typesupport_introspection_c__resize_function__JointWrench__wrenches  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers Contact__rosidl_typesupport_introspection_c__Contact_message_members = {
  "ros_ign_interfaces__msg",  // message namespace
  "Contact",  // message name
  6,  // number of fields
  sizeof(ros_ign_interfaces__msg__Contact),
  Contact__rosidl_typesupport_introspection_c__Contact_message_member_array,  // message members
  Contact__rosidl_typesupport_introspection_c__Contact_init_function,  // function to initialize message memory (memory has to be allocated)
  Contact__rosidl_typesupport_introspection_c__Contact_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t Contact__rosidl_typesupport_introspection_c__Contact_message_type_support_handle = {
  0,
  &Contact__rosidl_typesupport_introspection_c__Contact_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ros_ign_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros_ign_interfaces, msg, Contact)() {
  Contact__rosidl_typesupport_introspection_c__Contact_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros_ign_interfaces, msg, Entity)();
  Contact__rosidl_typesupport_introspection_c__Contact_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros_ign_interfaces, msg, Entity)();
  Contact__rosidl_typesupport_introspection_c__Contact_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Vector3)();
  Contact__rosidl_typesupport_introspection_c__Contact_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Vector3)();
  Contact__rosidl_typesupport_introspection_c__Contact_message_member_array[5].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros_ign_interfaces, msg, JointWrench)();
  if (!Contact__rosidl_typesupport_introspection_c__Contact_message_type_support_handle.typesupport_identifier) {
    Contact__rosidl_typesupport_introspection_c__Contact_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &Contact__rosidl_typesupport_introspection_c__Contact_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
