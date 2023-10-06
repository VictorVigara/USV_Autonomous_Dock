// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from ros_ign_interfaces:srv/SpawnEntity.idl
// generated code does not contain a copyright notice
#include "ros_ign_interfaces/srv/detail/spawn_entity__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "ros_ign_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "ros_ign_interfaces/srv/detail/spawn_entity__struct.h"
#include "ros_ign_interfaces/srv/detail/spawn_entity__functions.h"
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

#include "ros_ign_interfaces/msg/detail/entity_factory__functions.h"  // entity_factory

// forward declare type support functions
size_t get_serialized_size_ros_ign_interfaces__msg__EntityFactory(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_ros_ign_interfaces__msg__EntityFactory(
  bool & full_bounded,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ros_ign_interfaces, msg, EntityFactory)();


using _SpawnEntity_Request__ros_msg_type = ros_ign_interfaces__srv__SpawnEntity_Request;

static bool _SpawnEntity_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _SpawnEntity_Request__ros_msg_type * ros_message = static_cast<const _SpawnEntity_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: entity_factory
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, ros_ign_interfaces, msg, EntityFactory
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->entity_factory, cdr))
    {
      return false;
    }
  }

  return true;
}

static bool _SpawnEntity_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _SpawnEntity_Request__ros_msg_type * ros_message = static_cast<_SpawnEntity_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: entity_factory
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, ros_ign_interfaces, msg, EntityFactory
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->entity_factory))
    {
      return false;
    }
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros_ign_interfaces
size_t get_serialized_size_ros_ign_interfaces__srv__SpawnEntity_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _SpawnEntity_Request__ros_msg_type * ros_message = static_cast<const _SpawnEntity_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name entity_factory

  current_alignment += get_serialized_size_ros_ign_interfaces__msg__EntityFactory(
    &(ros_message->entity_factory), current_alignment);

  return current_alignment - initial_alignment;
}

static uint32_t _SpawnEntity_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_ros_ign_interfaces__srv__SpawnEntity_Request(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros_ign_interfaces
size_t max_serialized_size_ros_ign_interfaces__srv__SpawnEntity_Request(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: entity_factory
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_ros_ign_interfaces__msg__EntityFactory(
        full_bounded, current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

static size_t _SpawnEntity_Request__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_ros_ign_interfaces__srv__SpawnEntity_Request(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_SpawnEntity_Request = {
  "ros_ign_interfaces::srv",
  "SpawnEntity_Request",
  _SpawnEntity_Request__cdr_serialize,
  _SpawnEntity_Request__cdr_deserialize,
  _SpawnEntity_Request__get_serialized_size,
  _SpawnEntity_Request__max_serialized_size
};

static rosidl_message_type_support_t _SpawnEntity_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_SpawnEntity_Request,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ros_ign_interfaces, srv, SpawnEntity_Request)() {
  return &_SpawnEntity_Request__type_support;
}

#if defined(__cplusplus)
}
#endif

// already included above
// #include <cassert>
// already included above
// #include <limits>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "ros_ign_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "ros_ign_interfaces/srv/detail/spawn_entity__struct.h"
// already included above
// #include "ros_ign_interfaces/srv/detail/spawn_entity__functions.h"
// already included above
// #include "fastcdr/Cdr.h"

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


// forward declare type support functions


using _SpawnEntity_Response__ros_msg_type = ros_ign_interfaces__srv__SpawnEntity_Response;

static bool _SpawnEntity_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _SpawnEntity_Response__ros_msg_type * ros_message = static_cast<const _SpawnEntity_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: success
  {
    cdr << (ros_message->success ? true : false);
  }

  return true;
}

static bool _SpawnEntity_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _SpawnEntity_Response__ros_msg_type * ros_message = static_cast<_SpawnEntity_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: success
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->success = tmp ? true : false;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros_ign_interfaces
size_t get_serialized_size_ros_ign_interfaces__srv__SpawnEntity_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _SpawnEntity_Response__ros_msg_type * ros_message = static_cast<const _SpawnEntity_Response__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name success
  {
    size_t item_size = sizeof(ros_message->success);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _SpawnEntity_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_ros_ign_interfaces__srv__SpawnEntity_Response(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros_ign_interfaces
size_t max_serialized_size_ros_ign_interfaces__srv__SpawnEntity_Response(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: success
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  return current_alignment - initial_alignment;
}

static size_t _SpawnEntity_Response__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_ros_ign_interfaces__srv__SpawnEntity_Response(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_SpawnEntity_Response = {
  "ros_ign_interfaces::srv",
  "SpawnEntity_Response",
  _SpawnEntity_Response__cdr_serialize,
  _SpawnEntity_Response__cdr_deserialize,
  _SpawnEntity_Response__get_serialized_size,
  _SpawnEntity_Response__max_serialized_size
};

static rosidl_message_type_support_t _SpawnEntity_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_SpawnEntity_Response,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ros_ign_interfaces, srv, SpawnEntity_Response)() {
  return &_SpawnEntity_Response__type_support;
}

#if defined(__cplusplus)
}
#endif

#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "ros_ign_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "ros_ign_interfaces/srv/spawn_entity.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t SpawnEntity__callbacks = {
  "ros_ign_interfaces::srv",
  "SpawnEntity",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ros_ign_interfaces, srv, SpawnEntity_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ros_ign_interfaces, srv, SpawnEntity_Response)(),
};

static rosidl_service_type_support_t SpawnEntity__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &SpawnEntity__callbacks,
  get_service_typesupport_handle_function,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ros_ign_interfaces, srv, SpawnEntity)() {
  return &SpawnEntity__handle;
}

#if defined(__cplusplus)
}
#endif
