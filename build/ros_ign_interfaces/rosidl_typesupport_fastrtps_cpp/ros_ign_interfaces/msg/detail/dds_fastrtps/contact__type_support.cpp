// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from ros_ign_interfaces:msg/Contact.idl
// generated code does not contain a copyright notice
#include "ros_ign_interfaces/msg/detail/contact__rosidl_typesupport_fastrtps_cpp.hpp"
#include "ros_ign_interfaces/msg/detail/contact__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions
namespace ros_ign_interfaces
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const ros_ign_interfaces::msg::Entity &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  ros_ign_interfaces::msg::Entity &);
size_t get_serialized_size(
  const ros_ign_interfaces::msg::Entity &,
  size_t current_alignment);
size_t
max_serialized_size_Entity(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace ros_ign_interfaces

namespace ros_ign_interfaces
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const ros_ign_interfaces::msg::Entity &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  ros_ign_interfaces::msg::Entity &);
size_t get_serialized_size(
  const ros_ign_interfaces::msg::Entity &,
  size_t current_alignment);
size_t
max_serialized_size_Entity(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace ros_ign_interfaces

namespace geometry_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const geometry_msgs::msg::Vector3 &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  geometry_msgs::msg::Vector3 &);
size_t get_serialized_size(
  const geometry_msgs::msg::Vector3 &,
  size_t current_alignment);
size_t
max_serialized_size_Vector3(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace geometry_msgs

namespace geometry_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const geometry_msgs::msg::Vector3 &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  geometry_msgs::msg::Vector3 &);
size_t get_serialized_size(
  const geometry_msgs::msg::Vector3 &,
  size_t current_alignment);
size_t
max_serialized_size_Vector3(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace geometry_msgs

namespace ros_ign_interfaces
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const ros_ign_interfaces::msg::JointWrench &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  ros_ign_interfaces::msg::JointWrench &);
size_t get_serialized_size(
  const ros_ign_interfaces::msg::JointWrench &,
  size_t current_alignment);
size_t
max_serialized_size_JointWrench(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace ros_ign_interfaces


namespace ros_ign_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ros_ign_interfaces
cdr_serialize(
  const ros_ign_interfaces::msg::Contact & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: collision1
  ros_ign_interfaces::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.collision1,
    cdr);
  // Member: collision2
  ros_ign_interfaces::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.collision2,
    cdr);
  // Member: positions
  {
    size_t size = ros_message.positions.size();
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; i++) {
      geometry_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
        ros_message.positions[i],
        cdr);
    }
  }
  // Member: normals
  {
    size_t size = ros_message.normals.size();
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; i++) {
      geometry_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
        ros_message.normals[i],
        cdr);
    }
  }
  // Member: depths
  {
    cdr << ros_message.depths;
  }
  // Member: wrenches
  {
    size_t size = ros_message.wrenches.size();
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; i++) {
      ros_ign_interfaces::msg::typesupport_fastrtps_cpp::cdr_serialize(
        ros_message.wrenches[i],
        cdr);
    }
  }
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ros_ign_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  ros_ign_interfaces::msg::Contact & ros_message)
{
  // Member: collision1
  ros_ign_interfaces::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.collision1);

  // Member: collision2
  ros_ign_interfaces::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.collision2);

  // Member: positions
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    ros_message.positions.resize(size);
    for (size_t i = 0; i < size; i++) {
      geometry_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
        cdr, ros_message.positions[i]);
    }
  }

  // Member: normals
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    ros_message.normals.resize(size);
    for (size_t i = 0; i < size; i++) {
      geometry_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
        cdr, ros_message.normals[i]);
    }
  }

  // Member: depths
  {
    cdr >> ros_message.depths;
  }

  // Member: wrenches
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    ros_message.wrenches.resize(size);
    for (size_t i = 0; i < size; i++) {
      ros_ign_interfaces::msg::typesupport_fastrtps_cpp::cdr_deserialize(
        cdr, ros_message.wrenches[i]);
    }
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ros_ign_interfaces
get_serialized_size(
  const ros_ign_interfaces::msg::Contact & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: collision1

  current_alignment +=
    ros_ign_interfaces::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.collision1, current_alignment);
  // Member: collision2

  current_alignment +=
    ros_ign_interfaces::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.collision2, current_alignment);
  // Member: positions
  {
    size_t array_size = ros_message.positions.size();

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        geometry_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
        ros_message.positions[index], current_alignment);
    }
  }
  // Member: normals
  {
    size_t array_size = ros_message.normals.size();

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        geometry_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
        ros_message.normals[index], current_alignment);
    }
  }
  // Member: depths
  {
    size_t array_size = ros_message.depths.size();

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    size_t item_size = sizeof(ros_message.depths[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: wrenches
  {
    size_t array_size = ros_message.wrenches.size();

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        ros_ign_interfaces::msg::typesupport_fastrtps_cpp::get_serialized_size(
        ros_message.wrenches[index], current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ros_ign_interfaces
max_serialized_size_Contact(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: collision1
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        ros_ign_interfaces::msg::typesupport_fastrtps_cpp::max_serialized_size_Entity(
        full_bounded, current_alignment);
    }
  }

  // Member: collision2
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        ros_ign_interfaces::msg::typesupport_fastrtps_cpp::max_serialized_size_Entity(
        full_bounded, current_alignment);
    }
  }

  // Member: positions
  {
    size_t array_size = 0;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        geometry_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Vector3(
        full_bounded, current_alignment);
    }
  }

  // Member: normals
  {
    size_t array_size = 0;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        geometry_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Vector3(
        full_bounded, current_alignment);
    }
  }

  // Member: depths
  {
    size_t array_size = 0;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: wrenches
  {
    size_t array_size = 0;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        ros_ign_interfaces::msg::typesupport_fastrtps_cpp::max_serialized_size_JointWrench(
        full_bounded, current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

static bool _Contact__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const ros_ign_interfaces::msg::Contact *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _Contact__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<ros_ign_interfaces::msg::Contact *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _Contact__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const ros_ign_interfaces::msg::Contact *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _Contact__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_Contact(full_bounded, 0);
}

static message_type_support_callbacks_t _Contact__callbacks = {
  "ros_ign_interfaces::msg",
  "Contact",
  _Contact__cdr_serialize,
  _Contact__cdr_deserialize,
  _Contact__get_serialized_size,
  _Contact__max_serialized_size
};

static rosidl_message_type_support_t _Contact__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_Contact__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace ros_ign_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_ros_ign_interfaces
const rosidl_message_type_support_t *
get_message_type_support_handle<ros_ign_interfaces::msg::Contact>()
{
  return &ros_ign_interfaces::msg::typesupport_fastrtps_cpp::_Contact__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, ros_ign_interfaces, msg, Contact)() {
  return &ros_ign_interfaces::msg::typesupport_fastrtps_cpp::_Contact__handle;
}

#ifdef __cplusplus
}
#endif
