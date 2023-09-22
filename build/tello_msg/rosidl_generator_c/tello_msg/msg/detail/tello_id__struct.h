// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from tello_msg:msg/TelloID.idl
// generated code does not contain a copyright notice

#ifndef TELLO_MSG__MSG__DETAIL__TELLO_ID__STRUCT_H_
#define TELLO_MSG__MSG__DETAIL__TELLO_ID__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'sdk_version'
// Member 'serial_number'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/TelloID in the package tello_msg.
/**
  * SDK version
 */
typedef struct tello_msg__msg__TelloID
{
  rosidl_runtime_c__String sdk_version;
  /// Serial number
  rosidl_runtime_c__String serial_number;
} tello_msg__msg__TelloID;

// Struct for a sequence of tello_msg__msg__TelloID.
typedef struct tello_msg__msg__TelloID__Sequence
{
  tello_msg__msg__TelloID * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} tello_msg__msg__TelloID__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TELLO_MSG__MSG__DETAIL__TELLO_ID__STRUCT_H_
