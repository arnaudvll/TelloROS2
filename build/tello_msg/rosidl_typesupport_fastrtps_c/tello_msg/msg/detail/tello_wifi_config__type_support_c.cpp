// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from tello_msg:msg/TelloWifiConfig.idl
// generated code does not contain a copyright notice
#include "tello_msg/msg/detail/tello_wifi_config__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "tello_msg/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "tello_msg/msg/detail/tello_wifi_config__struct.h"
#include "tello_msg/msg/detail/tello_wifi_config__functions.h"
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

#include "rosidl_runtime_c/string.h"  // password, ssid
#include "rosidl_runtime_c/string_functions.h"  // password, ssid

// forward declare type support functions


using _TelloWifiConfig__ros_msg_type = tello_msg__msg__TelloWifiConfig;

static bool _TelloWifiConfig__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _TelloWifiConfig__ros_msg_type * ros_message = static_cast<const _TelloWifiConfig__ros_msg_type *>(untyped_ros_message);
  // Field name: ssid
  {
    const rosidl_runtime_c__String * str = &ros_message->ssid;
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

  // Field name: password
  {
    const rosidl_runtime_c__String * str = &ros_message->password;
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

static bool _TelloWifiConfig__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _TelloWifiConfig__ros_msg_type * ros_message = static_cast<_TelloWifiConfig__ros_msg_type *>(untyped_ros_message);
  // Field name: ssid
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->ssid.data) {
      rosidl_runtime_c__String__init(&ros_message->ssid);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->ssid,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'ssid'\n");
      return false;
    }
  }

  // Field name: password
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->password.data) {
      rosidl_runtime_c__String__init(&ros_message->password);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->password,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'password'\n");
      return false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_tello_msg
size_t get_serialized_size_tello_msg__msg__TelloWifiConfig(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _TelloWifiConfig__ros_msg_type * ros_message = static_cast<const _TelloWifiConfig__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name ssid
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->ssid.size + 1);
  // field.name password
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->password.size + 1);

  return current_alignment - initial_alignment;
}

static uint32_t _TelloWifiConfig__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_tello_msg__msg__TelloWifiConfig(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_tello_msg
size_t max_serialized_size_tello_msg__msg__TelloWifiConfig(
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

  // member: ssid
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
  // member: password
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

static size_t _TelloWifiConfig__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_tello_msg__msg__TelloWifiConfig(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_TelloWifiConfig = {
  "tello_msg::msg",
  "TelloWifiConfig",
  _TelloWifiConfig__cdr_serialize,
  _TelloWifiConfig__cdr_deserialize,
  _TelloWifiConfig__get_serialized_size,
  _TelloWifiConfig__max_serialized_size
};

static rosidl_message_type_support_t _TelloWifiConfig__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_TelloWifiConfig,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, tello_msg, msg, TelloWifiConfig)() {
  return &_TelloWifiConfig__type_support;
}

#if defined(__cplusplus)
}
#endif
