// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from tello_msg:msg/TelloWifiConfig.idl
// generated code does not contain a copyright notice

#ifndef TELLO_MSG__MSG__DETAIL__TELLO_WIFI_CONFIG__STRUCT_H_
#define TELLO_MSG__MSG__DETAIL__TELLO_WIFI_CONFIG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'ssid'
// Member 'password'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/TelloWifiConfig in the package tello_msg.
/**
  * Wifi SSID name
 */
typedef struct tello_msg__msg__TelloWifiConfig
{
  rosidl_runtime_c__String ssid;
  /// Wifi network password
  rosidl_runtime_c__String password;
} tello_msg__msg__TelloWifiConfig;

// Struct for a sequence of tello_msg__msg__TelloWifiConfig.
typedef struct tello_msg__msg__TelloWifiConfig__Sequence
{
  tello_msg__msg__TelloWifiConfig * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} tello_msg__msg__TelloWifiConfig__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TELLO_MSG__MSG__DETAIL__TELLO_WIFI_CONFIG__STRUCT_H_
