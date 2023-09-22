// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from tello_msg:msg/TelloStatus.idl
// generated code does not contain a copyright notice

#ifndef TELLO_MSG__MSG__DETAIL__TELLO_STATUS__FUNCTIONS_H_
#define TELLO_MSG__MSG__DETAIL__TELLO_STATUS__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "tello_msg/msg/rosidl_generator_c__visibility_control.h"

#include "tello_msg/msg/detail/tello_status__struct.h"

/// Initialize msg/TelloStatus message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * tello_msg__msg__TelloStatus
 * )) before or use
 * tello_msg__msg__TelloStatus__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_tello_msg
bool
tello_msg__msg__TelloStatus__init(tello_msg__msg__TelloStatus * msg);

/// Finalize msg/TelloStatus message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tello_msg
void
tello_msg__msg__TelloStatus__fini(tello_msg__msg__TelloStatus * msg);

/// Create msg/TelloStatus message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * tello_msg__msg__TelloStatus__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_tello_msg
tello_msg__msg__TelloStatus *
tello_msg__msg__TelloStatus__create();

/// Destroy msg/TelloStatus message.
/**
 * It calls
 * tello_msg__msg__TelloStatus__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tello_msg
void
tello_msg__msg__TelloStatus__destroy(tello_msg__msg__TelloStatus * msg);

/// Check for msg/TelloStatus message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_tello_msg
bool
tello_msg__msg__TelloStatus__are_equal(const tello_msg__msg__TelloStatus * lhs, const tello_msg__msg__TelloStatus * rhs);

/// Copy a msg/TelloStatus message.
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
tello_msg__msg__TelloStatus__copy(
  const tello_msg__msg__TelloStatus * input,
  tello_msg__msg__TelloStatus * output);

/// Initialize array of msg/TelloStatus messages.
/**
 * It allocates the memory for the number of elements and calls
 * tello_msg__msg__TelloStatus__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_tello_msg
bool
tello_msg__msg__TelloStatus__Sequence__init(tello_msg__msg__TelloStatus__Sequence * array, size_t size);

/// Finalize array of msg/TelloStatus messages.
/**
 * It calls
 * tello_msg__msg__TelloStatus__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tello_msg
void
tello_msg__msg__TelloStatus__Sequence__fini(tello_msg__msg__TelloStatus__Sequence * array);

/// Create array of msg/TelloStatus messages.
/**
 * It allocates the memory for the array and calls
 * tello_msg__msg__TelloStatus__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_tello_msg
tello_msg__msg__TelloStatus__Sequence *
tello_msg__msg__TelloStatus__Sequence__create(size_t size);

/// Destroy array of msg/TelloStatus messages.
/**
 * It calls
 * tello_msg__msg__TelloStatus__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tello_msg
void
tello_msg__msg__TelloStatus__Sequence__destroy(tello_msg__msg__TelloStatus__Sequence * array);

/// Check for msg/TelloStatus message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_tello_msg
bool
tello_msg__msg__TelloStatus__Sequence__are_equal(const tello_msg__msg__TelloStatus__Sequence * lhs, const tello_msg__msg__TelloStatus__Sequence * rhs);

/// Copy an array of msg/TelloStatus messages.
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
tello_msg__msg__TelloStatus__Sequence__copy(
  const tello_msg__msg__TelloStatus__Sequence * input,
  tello_msg__msg__TelloStatus__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // TELLO_MSG__MSG__DETAIL__TELLO_STATUS__FUNCTIONS_H_
