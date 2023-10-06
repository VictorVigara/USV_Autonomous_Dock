// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from ros_ign_interfaces:msg/EntityFactory.idl
// generated code does not contain a copyright notice
#include "ros_ign_interfaces/msg/detail/entity_factory__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "ros_ign_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "ros_ign_interfaces/msg/detail/entity_factory__struct.h"
#include "ros_ign_interfaces/msg/detail/entity_factory__functions.h"
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

#include "geometry_msgs/msg/detail/pose__functions.h"  // pose
#include "rosidl_runtime_c/string.h"  // clone_name, name, relative_to, sdf, sdf_filename
#include "rosidl_runtime_c/string_functions.h"  // clone_name, name, relative_to, sdf, sdf_filename

// forward declare type support functions
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ros_ign_interfaces
size_t get_serialized_size_geometry_msgs__msg__Pose(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ros_ign_interfaces
size_t max_serialized_size_geometry_msgs__msg__Pose(
  bool & full_bounded,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ros_ign_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Pose)();


using _EntityFactory__ros_msg_type = ros_ign_interfaces__msg__EntityFactory;

static bool _EntityFactory__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _EntityFactory__ros_msg_type * ros_message = static_cast<const _EntityFactory__ros_msg_type *>(untyped_ros_message);
  // Field name: name
  {
    const rosidl_runtime_c__String * str = &ros_message->name;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: allow_renaming
  {
    cdr << (ros_message->allow_renaming ? true : false);
  }

  // Field name: sdf
  {
    const rosidl_runtime_c__String * str = &ros_message->sdf;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: sdf_filename
  {
    const rosidl_runtime_c__String * str = &ros_message->sdf_filename;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: clone_name
  {
    const rosidl_runtime_c__String * str = &ros_message->clone_name;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: pose
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Pose
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->pose, cdr))
    {
      return false;
    }
  }

  // Field name: relative_to
  {
    const rosidl_runtime_c__String * str = &ros_message->relative_to;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  return true;
}

static bool _EntityFactory__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _EntityFactory__ros_msg_type * ros_message = static_cast<_EntityFactory__ros_msg_type *>(untyped_ros_message);
  // Field name: name
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->name.data) {
      rosidl_runtime_c__String__init(&ros_message->name);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->name,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'name'\n");
      return false;
    }
  }

  // Field name: allow_renaming
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->allow_renaming = tmp ? true : false;
  }

  // Field name: sdf
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->sdf.data) {
      rosidl_runtime_c__String__init(&ros_message->sdf);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->sdf,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'sdf'\n");
      return false;
    }
  }

  // Field name: sdf_filename
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->sdf_filename.data) {
      rosidl_runtime_c__String__init(&ros_message->sdf_filename);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->sdf_filename,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'sdf_filename'\n");
      return false;
    }
  }

  // Field name: clone_name
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->clone_name.data) {
      rosidl_runtime_c__String__init(&ros_message->clone_name);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->clone_name,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'clone_name'\n");
      return false;
    }
  }

  // Field name: pose
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Pose
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->pose))
    {
      return false;
    }
  }

  // Field name: relative_to
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->relative_to.data) {
      rosidl_runtime_c__String__init(&ros_message->relative_to);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->relative_to,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'relative_to'\n");
      return false;
    }
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros_ign_interfaces
size_t get_serialized_size_ros_ign_interfaces__msg__EntityFactory(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _EntityFactory__ros_msg_type * ros_message = static_cast<const _EntityFactory__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name name
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->name.size + 1);
  // field.name allow_renaming
  {
    size_t item_size = sizeof(ros_message->allow_renaming);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name sdf
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->sdf.size + 1);
  // field.name sdf_filename
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->sdf_filename.size + 1);
  // field.name clone_name
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->clone_name.size + 1);
  // field.name pose

  current_alignment += get_serialized_size_geometry_msgs__msg__Pose(
    &(ros_message->pose), current_alignment);
  // field.name relative_to
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->relative_to.size + 1);

  return current_alignment - initial_alignment;
}

static uint32_t _EntityFactory__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_ros_ign_interfaces__msg__EntityFactory(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros_ign_interfaces
size_t max_serialized_size_ros_ign_interfaces__msg__EntityFactory(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: name
  {
    size_t array_size = 1;

    full_bounded = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: allow_renaming
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: sdf
  {
    size_t array_size = 1;

    full_bounded = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: sdf_filename
  {
    size_t array_size = 1;

    full_bounded = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: clone_name
  {
    size_t array_size = 1;

    full_bounded = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: pose
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_geometry_msgs__msg__Pose(
        full_bounded, current_alignment);
    }
  }
  // member: relative_to
  {
    size_t array_size = 1;

    full_bounded = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  return current_alignment - initial_alignment;
}

static size_t _EntityFactory__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_ros_ign_interfaces__msg__EntityFactory(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_EntityFactory = {
  "ros_ign_interfaces::msg",
  "EntityFactory",
  _EntityFactory__cdr_serialize,
  _EntityFactory__cdr_deserialize,
  _EntityFactory__get_serialized_size,
  _EntityFactory__max_serialized_size
};

static rosidl_message_type_support_t _EntityFactory__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_EntityFactory,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ros_ign_interfaces, msg, EntityFactory)() {
  return &_EntityFactory__type_support;
}

#if defined(__cplusplus)
}
#endif
