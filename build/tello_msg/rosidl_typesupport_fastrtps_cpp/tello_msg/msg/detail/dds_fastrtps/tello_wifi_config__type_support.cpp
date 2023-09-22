// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from tello_msg:msg/TelloWifiConfig.idl
// generated code does not contain a copyright notice
#include "tello_msg/msg/detail/tello_wifi_config__rosidl_typesupport_fastrtps_cpp.hpp"
#include "tello_msg/msg/detail/tello_wifi_config__struct.hpp"

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

namespace tello_msg
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_tello_msg
cdr_serialize(
  const tello_msg::msg::TelloWifiConfig & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: ssid
  cdr << ros_message.ssid;
  // Member: password
  cdr << ros_message.password;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_tello_msg
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  tello_msg::msg::TelloWifiConfig & ros_message)
{
  // Member: ssid
  cdr >> ros_message.ssid;

  // Member: password
  cdr >> ros_message.password;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_tello_msg
get_serialized_size(
  const tello_msg::msg::TelloWifiConfig & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: ssid
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.ssid.size() + 1);
  // Member: password
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.password.size() + 1);

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_tello_msg
max_serialized_size_TelloWifiConfig(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: ssid
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Member: password
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  return current_alignment - initial_alignment;
}

static bool _TelloWifiConfig__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const tello_msg::msg::TelloWifiConfig *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _TelloWifiConfig__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<tello_msg::msg::TelloWifiConfig *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _TelloWifiConfig__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const tello_msg::msg::TelloWifiConfig *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _TelloWifiConfig__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_TelloWifiConfig(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _TelloWifiConfig__callbacks = {
  "tello_msg::msg",
  "TelloWifiConfig",
  _TelloWifiConfig__cdr_serialize,
  _TelloWifiConfig__cdr_deserialize,
  _TelloWifiConfig__get_serialized_size,
  _TelloWifiConfig__max_serialized_size
};

static rosidl_message_type_support_t _TelloWifiConfig__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_TelloWifiConfig__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace tello_msg

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_tello_msg
const rosidl_message_type_support_t *
get_message_type_support_handle<tello_msg::msg::TelloWifiConfig>()
{
  return &tello_msg::msg::typesupport_fastrtps_cpp::_TelloWifiConfig__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, tello_msg, msg, TelloWifiConfig)() {
  return &tello_msg::msg::typesupport_fastrtps_cpp::_TelloWifiConfig__handle;
}

#ifdef __cplusplus
}
#endif
