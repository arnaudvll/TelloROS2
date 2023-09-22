// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from tello_msg:msg/TelloWifiConfig.idl
// generated code does not contain a copyright notice

#ifndef TELLO_MSG__MSG__DETAIL__TELLO_WIFI_CONFIG__FUNCTIONS_H_
#define TELLO_MSG__MSG__DETAIL__TELLO_WIFI_CONFIG__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "tello_msg/msg/rosidl_generator_c__visibility_control.h"

#include "tello_msg/msg/detail/tello_wifi_config__struct.h"

/// Initialize msg/TelloWifiConfig message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * tello_msg__msg__TelloWifiConfig
 * )) before or use
 * tello_msg__msg__TelloWifiConfig__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_tello_msg
bool
tello_msg__msg__TelloWifiConfig__init(tello_msg__msg__TelloWifiConfig * msg);

/// Finalize msg/TelloWifiConfig message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tello_msg
void
tello_msg__msg__TelloWifiConfig__fini(tello_msg__msg__TelloWifiConfig * msg);

/// Create msg/TelloWifiConfig message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * tello_msg__msg__TelloWifiConfig__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_tello_msg
tello_msg__msg__TelloWifiConfig *
tello_msg__msg__TelloWifiConfig__create();

/// Destroy msg/TelloWifiConfig message.
/**
 * It calls
 * tello_msg__msg__TelloWifiConfig__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tello_msg
void
tello_msg__msg__TelloWifiConfig__destroy(tello_msg__msg__TelloWifiConfig * msg);

/// Check for msg/TelloWifiConfig message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_tello_msg
bool
tello_msg__msg__TelloWifiConfig__are_equal(const tello_msg__msg__TelloWifiConfig * lhs, const tello_msg__msg__TelloWifiConfig * rhs);

/// Copy a msg/TelloWifiConfig message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_tello_msg
bool
tello_msg__msg__TelloWifiConfig__copy(
  const tello_msg__msg__TelloWifiConfig * input,
  tello_msg__msg__TelloWifiConfig * output);

/// Initialize array of msg/TelloWifiConfig messages.
/**
 * It allocates the memory for the number of elements and calls
 * tello_msg__msg__TelloWifiConfig__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_tello_msg
bool
tello_msg__msg__TelloWifiConfig__Sequence__init(tello_msg__msg__TelloWifiConfig__Sequence * array, size_t size);

/// Finalize array of msg/TelloWifiConfig messages.
/**
 * It calls
 * tello_msg__msg__TelloWifiConfig__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tello_msg
void
tello_msg__msg__TelloWifiConfig__Sequence__fini(tello_msg__msg__TelloWifiConfig__Sequence * array);

/// Create array of msg/TelloWifiConfig messages.
/**
 * It allocates the memory for the array and calls
 * tello_msg__msg__TelloWifiConfig__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_tello_msg
tello_msg__msg__TelloWifiConfig__Sequence *
tello_msg__msg__TelloWifiConfig__Sequence__create(size_t size);

/// Destroy array of msg/TelloWifiConfig messages.
/**
 * It calls
 * tello_msg__msg__TelloWifiConfig__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tello_msg
void
tello_msg__msg__TelloWifiConfig__Sequence__destroy(tello_msg__msg__TelloWifiConfig__Sequence * array);

/// Check for msg/TelloWifiConfig message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_tello_msg
bool
tello_msg__msg__TelloWifiConfig__Sequence__are_equal(const tello_msg__msg__TelloWifiConfig__Sequence * lhs, const tello_msg__msg__TelloWifiConfig__Sequence * rhs);

/// Copy an array of msg/TelloWifiConfig messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_tello_msg
bool
tello_msg__msg__TelloWifiConfig__Sequence__copy(
  const tello_msg__msg__TelloWifiConfig__Sequence * input,
  tello_msg__msg__TelloWifiConfig__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // TELLO_MSG__MSG__DETAIL__TELLO_WIFI_CONFIG__FUNCTIONS_H_
