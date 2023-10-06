// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from ros_ign_interfaces:msg/JointWrench.idl
// generated code does not contain a copyright notice
#include "ros_ign_interfaces/msg/detail/joint_wrench__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "ros_ign_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "ros_ign_interfaces/msg/detail/joint_wrench__struct.h"
#include "ros_ign_interfaces/msg/detail/joint_wrench__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "geometry_msgs/msg/detail/wrench__functions.h"  // body_1_wrench, body_2_wrench
#include "std_msgs/msg/detail/header__functions.h"  // header
#include "std_msgs/msg/detail/string__functions.h"  // body_1_name, body_2_name
#include "std_msgs/msg/detail/u_int32__functions.h"  // body_1_id, body_2_id

// forward declare type support functions
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ros_ign_interfaces
size_t get_serialized_size_geometry_msgs__msg__Wrench(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ros_ign_interfaces
size_t max_serialized_size_geometry_msgs__msg__Wrench(
  bool & full_bounded,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ros_ign_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Wrench)();
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ros_ign_interfaces
size_t get_serialized_size_std_msgs__msg__Header(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ros_ign_interfaces
size_t max_serialized_size_std_msgs__msg__Header(
  bool & full_bounded,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ros_ign_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, std_msgs, msg, Header)();
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ros_ign_interfaces
size_t get_serialized_size_std_msgs__msg__String(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ros_ign_interfaces
size_t max_serialized_size_std_msgs__msg__String(
  bool & full_bounded,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ros_ign_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, std_msgs, msg, String)();
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ros_ign_interfaces
size_t get_serialized_size_std_msgs__msg__UInt32(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ros_ign_interfaces
size_t max_serialized_size_std_msgs__msg__UInt32(
  bool & full_bounded,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ros_ign_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, std_msgs, msg, UInt32)();


using _JointWrench__ros_msg_type = ros_ign_interfaces__msg__JointWrench;

static bool _JointWrench__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _JointWrench__ros_msg_type * ros_message = static_cast<const _JointWrench__ros_msg_type *>(untyped_ros_message);
  // Field name: header
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, std_msgs, msg, Header
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->header, cdr))
    {
      return false;
    }
  }

  // Field name: body_1_name
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, std_msgs, msg, String
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->body_1_name, cdr))
    {
      return false;
    }
  }

  // Field name: body_1_id
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, std_msgs, msg, UInt32
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->body_1_id, cdr))
    {
      return false;
    }
  }

  // Field name: body_2_name
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, std_msgs, msg, String
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->body_2_name, cdr))
    {
      return false;
    }
  }

  // Field name: body_2_id
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, std_msgs, msg, UInt32
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->body_2_id, cdr))
    {
      return false;
    }
  }

  // Field name: body_1_wrench
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Wrench
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->body_1_wrench, cdr))
    {
      return false;
    }
  }

  // Field name: body_2_wrench
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Wrench
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->body_2_wrench, cdr))
    {
      return false;
    }
  }

  return true;
}

static bool _JointWrench__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _JointWrench__ros_msg_type * ros_message = static_cast<_JointWrench__ros_msg_type *>(untyped_ros_message);
  // Field name: header
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, std_msgs, msg, Header
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->header))
    {
      return false;
    }
  }

  // Field name: body_1_name
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, std_msgs, msg, String
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->body_1_name))
    {
      return false;
    }
  }

  // Field name: body_1_id
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, std_msgs, msg, UInt32
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->body_1_id))
    {
      return false;
    }
  }

  // Field name: body_2_name
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, std_msgs, msg, String
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->body_2_name))
    {
      return false;
    }
  }

  // Field name: body_2_id
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, std_msgs, msg, UInt32
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->body_2_id))
    {
      return false;
    }
  }

  // Field name: body_1_wrench
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Wrench
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->body_1_wrench))
    {
      return false;
    }
  }

  // Field name: body_2_wrench
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Wrench
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->body_2_wrench))
    {
      return false;
    }
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros_ign_interfaces
size_t get_serialized_size_ros_ign_interfaces__msg__JointWrench(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _JointWrench__ros_msg_type * ros_message = static_cast<const _JointWrench__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name header

  current_alignment += get_serialized_size_std_msgs__msg__Header(
    &(ros_message->header), current_alignment);
  // field.name body_1_name

  current_alignment += get_serialized_size_std_msgs__msg__String(
    &(ros_message->body_1_name), current_alignment);
  // field.name body_1_id

  current_alignment += get_serialized_size_std_msgs__msg__UInt32(
    &(ros_message->body_1_id), current_alignment);
  // field.name body_2_name

  current_alignment += get_serialized_size_std_msgs__msg__String(
    &(ros_message->body_2_name), current_alignment);
  // field.name body_2_id

  current_alignment += get_serialized_size_std_msgs__msg__UInt32(
    &(ros_message->body_2_id), current_alignment);
  // field.name body_1_wrench

  current_alignment += get_serialized_size_geometry_msgs__msg__Wrench(
    &(ros_message->body_1_wrench), current_alignment);
  // field.name body_2_wrench

  current_alignment += get_serialized_size_geometry_msgs__msg__Wrench(
    &(ros_message->body_2_wrench), current_alignment);

  return current_alignment - initial_alignment;
}

static uint32_t _JointWrench__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_ros_ign_interfaces__msg__JointWrench(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros_ign_interfaces
size_t max_serialized_size_ros_ign_interfaces__msg__JointWrench(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: header
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_std_msgs__msg__Header(
        full_bounded, current_alignment);
    }
  }
  // member: body_1_name
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_std_msgs__msg__String(
        full_bounded, current_alignment);
    }
  }
  // member: body_1_id
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_std_msgs__msg__UInt32(
        full_bounded, current_alignment);
    }
  }
  // member: body_2_name
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_std_msgs__msg__String(
        full_bounded, current_alignment);
    }
  }
  // member: body_2_id
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_std_msgs__msg__UInt32(
        full_bounded, current_alignment);
    }
  }
  // member: body_1_wrench
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_geometry_msgs__msg__Wrench(
        full_bounded, current_alignment);
    }
  }
  // member: body_2_wrench
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_geometry_msgs__msg__Wrench(
        full_bounded, current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

static size_t _JointWrench__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_ros_ign_interfaces__msg__JointWrench(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_JointWrench = {
  "ros_ign_interfaces::msg",
  "JointWrench",
  _JointWrench__cdr_serialize,
  _JointWrench__cdr_deserialize,
  _JointWrench__get_serialized_size,
  _JointWrench__max_serialized_size
};

static rosidl_message_type_support_t _JointWrench__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_JointWrench,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ros_ign_interfaces, msg, JointWrench)() {
  return &_JointWrench__type_support;
}

#if defined(__cplusplus)
}
#endif
