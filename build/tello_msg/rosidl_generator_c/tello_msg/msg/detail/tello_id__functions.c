// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from tello_msg:msg/TelloID.idl
// generated code does not contain a copyright notice
#include "tello_msg/msg/detail/tello_id__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `sdk_version`
// Member `serial_number`
#include "rosidl_runtime_c/string_functions.h"

bool
tello_msg__msg__TelloID__init(tello_msg__msg__TelloID * msg)
{
  if (!msg) {
    return false;
  }
  // sdk_version
  if (!rosidl_runtime_c__String__init(&msg->sdk_version)) {
    tello_msg__msg__TelloID__fini(msg);
    return false;
  }
  // serial_number
  if (!rosidl_runtime_c__String__init(&msg->serial_number)) {
    tello_msg__msg__TelloID__fini(msg);
    return false;
  }
  return true;
}

void
tello_msg__msg__TelloID__fini(tello_msg__msg__TelloID * msg)
{
  if (!msg) {
    return;
  }
  // sdk_version
  rosidl_runtime_c__String__fini(&msg->sdk_version);
  // serial_number
  rosidl_runtime_c__String__fini(&msg->serial_number);
}

bool
tello_msg__msg__TelloID__are_equal(const tello_msg__msg__TelloID * lhs, const tello_msg__msg__TelloID * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // sdk_version
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->sdk_version), &(rhs->sdk_version)))
  {
    return false;
  }
  // serial_number
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->serial_number), &(rhs->serial_number)))
  {
    return false;
  }
  return true;
}

bool
tello_msg__msg__TelloID__copy(
  const tello_msg__msg__TelloID * input,
  tello_msg__msg__TelloID * output)
{
  if (!input || !output) {
    return false;
  }
  // sdk_version
  if (!rosidl_runtime_c__String__copy(
      &(input->sdk_version), &(output->sdk_version)))
  {
    return false;
  }
  // serial_number
  if (!rosidl_runtime_c__String__copy(
      &(input->serial_number), &(output->serial_number)))
  {
    return false;
  }
  return true;
}

tello_msg__msg__TelloID *
tello_msg__msg__TelloID__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tello_msg__msg__TelloID * msg = (tello_msg__msg__TelloID *)allocator.allocate(sizeof(tello_msg__msg__TelloID), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(tello_msg__msg__TelloID));
  bool success = tello_msg__msg__TelloID__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
tello_msg__msg__TelloID__destroy(tello_msg__msg__TelloID * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    tello_msg__msg__TelloID__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
tello_msg__msg__TelloID__Sequence__init(tello_msg__msg__TelloID__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tello_msg__msg__TelloID * data = NULL;

  if (size) {
    data = (tello_msg__msg__TelloID *)allocator.zero_allocate(size, sizeof(tello_msg__msg__TelloID), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = tello_msg__msg__TelloID__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        tello_msg__msg__TelloID__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
tello_msg__msg__TelloID__Sequence__fini(tello_msg__msg__TelloID__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      tello_msg__msg__TelloID__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

tello_msg__msg__TelloID__Sequence *
tello_msg__msg__TelloID__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tello_msg__msg__TelloID__Sequence * array = (tello_msg__msg__TelloID__Sequence *)allocator.allocate(sizeof(tello_msg__msg__TelloID__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = tello_msg__msg__TelloID__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
tello_msg__msg__TelloID__Sequence__destroy(tello_msg__msg__TelloID__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    tello_msg__msg__TelloID__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
tello_msg__msg__TelloID__Sequence__are_equal(const tello_msg__msg__TelloID__Sequence * lhs, const tello_msg__msg__TelloID__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!tello_msg__msg__TelloID__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
tello_msg__msg__TelloID__Sequence__copy(
  const tello_msg__msg__TelloID__Sequence * input,
  tello_msg__msg__TelloID__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(tello_msg__msg__TelloID);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    tello_msg__msg__TelloID * data =
      (tello_msg__msg__TelloID *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!tello_msg__msg__TelloID__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          tello_msg__msg__TelloID__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!tello_msg__msg__TelloID__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
