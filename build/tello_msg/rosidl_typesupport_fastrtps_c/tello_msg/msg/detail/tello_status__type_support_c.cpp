// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from tello_msg:msg/TelloStatus.idl
// generated code does not contain a copyright notice
#include "tello_msg/msg/detail/tello_status__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "tello_msg/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "tello_msg/msg/detail/tello_status__struct.h"
#include "tello_msg/msg/detail/tello_status__functions.h"
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

#include "geometry_msgs/msg/detail/vector3__functions.h"  // acceleration, speed
#include "rosidl_runtime_c/string.h"  // wifi_snr
#include "rosidl_runtime_c/string_functions.h"  // wifi_snr

// forward declare type support functions
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_tello_msg
size_t get_serialized_size_geometry_msgs__msg__Vector3(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_tello_msg
size_t max_serialized_size_geometry_msgs__msg__Vector3(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_tello_msg
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Vector3)();


using _TelloStatus__ros_msg_type = tello_msg__msg__TelloStatus;

static bool _TelloStatus__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _TelloStatus__ros_msg_type * ros_message = static_cast<const _TelloStatus__ros_msg_type *>(untyped_ros_message);
  // Field name: acceleration
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Vector3
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->acceleration, cdr))
    {
      return false;
    }
  }

  // Field name: speed
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Vector3
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->speed, cdr))
    {
      return false;
    }
  }

  // Field name: barometer
  {
    cdr << ros_message->barometer;
  }

  // Field name: distance_tof
  {
    cdr << ros_message->distance_tof;
  }

  // Field name: battery
  {
    cdr << ros_message->battery;
  }

  // Field name: fligth_time
  {
    cdr << ros_message->fligth_time;
  }

  // Field name: highest_temperature
  {
    cdr << ros_message->highest_temperature;
  }

  // Field name: lowest_temperature
  {
    cdr << ros_message->lowest_temperature;
  }

  // Field name: temperature
  {
    cdr << ros_message->temperature;
  }

  // Field name: pitch
  {
    cdr << ros_message->pitch;
  }

  // Field name: roll
  {
    cdr << ros_message->roll;
  }

  // Field name: yaw
  {
    cdr << ros_message->yaw;
  }

  // Field name: wifi_snr
  {
    const rosidl_runtime_c__String * str = &ros_message->wifi_snr;
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

static bool _TelloStatus__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _TelloStatus__ros_msg_type * ros_message = static_cast<_TelloStatus__ros_msg_type *>(untyped_ros_message);
  // Field name: acceleration
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Vector3
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->acceleration))
    {
      return false;
    }
  }

  // Field name: speed
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Vector3
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->speed))
    {
      return false;
    }
  }

  // Field name: barometer
  {
    cdr >> ros_message->barometer;
  }

  // Field name: distance_tof
  {
    cdr >> ros_message->distance_tof;
  }

  // Field name: battery
  {
    cdr >> ros_message->battery;
  }

  // Field name: fligth_time
  {
    cdr >> ros_message->fligth_time;
  }

  // Field name: highest_temperature
  {
    cdr >> ros_message->highest_temperature;
  }

  // Field name: lowest_temperature
  {
    cdr >> ros_message->lowest_temperature;
  }

  // Field name: temperature
  {
    cdr >> ros_message->temperature;
  }

  // Field name: pitch
  {
    cdr >> ros_message->pitch;
  }

  // Field name: roll
  {
    cdr >> ros_message->roll;
  }

  // Field name: yaw
  {
    cdr >> ros_message->yaw;
  }

  // Field name: wifi_snr
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->wifi_snr.data) {
      rosidl_runtime_c__String__init(&ros_message->wifi_snr);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->wifi_snr,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'wifi_snr'\n");
      return false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_tello_msg
size_t get_serialized_size_tello_msg__msg__TelloStatus(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _TelloStatus__ros_msg_type * ros_message = static_cast<const _TelloStatus__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name acceleration

  current_alignment += get_serialized_size_geometry_msgs__msg__Vector3(
    &(ros_message->acceleration), current_alignment);
  // field.name speed

  current_alignment += get_serialized_size_geometry_msgs__msg__Vector3(
    &(ros_message->speed), current_alignment);
  // field.name barometer
  {
    size_t item_size = sizeof(ros_message->barometer);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name distance_tof
  {
    size_t item_size = sizeof(ros_message->distance_tof);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name battery
  {
    size_t item_size = sizeof(ros_message->battery);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name fligth_time
  {
    size_t item_size = sizeof(ros_message->fligth_time);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name highest_temperature
  {
    size_t item_size = sizeof(ros_message->highest_temperature);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name lowest_temperature
  {
    size_t item_size = sizeof(ros_message->lowest_temperature);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name temperature
  {
    size_t item_size = sizeof(ros_message->temperature);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name pitch
  {
    size_t item_size = sizeof(ros_message->pitch);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name roll
  {
    size_t item_size = sizeof(ros_message->roll);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name yaw
  {
    size_t item_size = sizeof(ros_message->yaw);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name wifi_snr
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->wifi_snr.size + 1);

  return current_alignment - initial_alignment;
}

static uint32_t _TelloStatus__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_tello_msg__msg__TelloStatus(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_tello_msg
size_t max_serialized_size_tello_msg__msg__TelloStatus(
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

  // member: acceleration
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      current_alignment +=
        max_serialized_size_geometry_msgs__msg__Vector3(
        inner_full_bounded, inner_is_plain, current_alignment);
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: speed
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      current_alignment +=
        max_serialized_size_geometry_msgs__msg__Vector3(
        inner_full_bounded, inner_is_plain, current_alignment);
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: barometer
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: distance_tof
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: battery
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: fligth_time
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: highest_temperature
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: lowest_temperature
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: temperature
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: pitch
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: roll
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: yaw
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: wifi_snr
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

static size_t _TelloStatus__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_tello_msg__msg__TelloStatus(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_TelloStatus = {
  "tello_msg::msg",
  "TelloStatus",
  _TelloStatus__cdr_serialize,
  _TelloStatus__cdr_deserialize,
  _TelloStatus__get_serialized_size,
  _TelloStatus__max_serialized_size
};

static rosidl_message_type_support_t _TelloStatus__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_TelloStatus,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, tello_msg, msg, TelloStatus)() {
  return &_TelloStatus__type_support;
}

#if defined(__cplusplus)
}
#endif
