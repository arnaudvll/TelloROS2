// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from tello_msg:msg/TelloStatus.idl
// generated code does not contain a copyright notice
#include "tello_msg/msg/detail/tello_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `acceleration`
// Member `speed`
#include "geometry_msgs/msg/detail/vector3__functions.h"
// Member `wifi_snr`
#include "rosidl_runtime_c/string_functions.h"

bool
tello_msg__msg__TelloStatus__init(tello_msg__msg__TelloStatus * msg)
{
  if (!msg) {
    return false;
  }
  // acceleration
  if (!geometry_msgs__msg__Vector3__init(&msg->acceleration)) {
    tello_msg__msg__TelloStatus__fini(msg);
    return false;
  }
  // speed
  if (!geometry_msgs__msg__Vector3__init(&msg->speed)) {
    tello_msg__msg__TelloStatus__fini(msg);
    return false;
  }
  // barometer
  // distance_tof
  // battery
  // fligth_time
  // highest_temperature
  // lowest_temperature
  // temperature
  // pitch
  // roll
  // yaw
  // wifi_snr
  if (!rosidl_runtime_c__String__init(&msg->wifi_snr)) {
    tello_msg__msg__TelloStatus__fini(msg);
    return false;
  }
  return true;
}

void
tello_msg__msg__TelloStatus__fini(tello_msg__msg__TelloStatus * msg)
{
  if (!msg) {
    return;
  }
  // acceleration
  geometry_msgs__msg__Vector3__fini(&msg->acceleration);
  // speed
  geometry_msgs__msg__Vector3__fini(&msg->speed);
  // barometer
  // distance_tof
  // battery
  // fligth_time
  // highest_temperature
  // lowest_temperature
  // temperature
  // pitch
  // roll
  // yaw
  // wifi_snr
  rosidl_runtime_c__String__fini(&msg->wifi_snr);
}

bool
tello_msg__msg__TelloStatus__are_equal(const tello_msg__msg__TelloStatus * lhs, const tello_msg__msg__TelloStatus * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // acceleration
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->acceleration), &(rhs->acceleration)))
  {
    return false;
  }
  // speed
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->speed), &(rhs->speed)))
  {
    return false;
  }
  // barometer
  if (lhs->barometer != rhs->barometer) {
    return false;
  }
  // distance_tof
  if (lhs->distance_tof != rhs->distance_tof) {
    return false;
  }
  // battery
  if (lhs->battery != rhs->battery) {
    return false;
  }
  // fligth_time
  if (lhs->fligth_time != rhs->fligth_time) {
    return false;
  }
  // highest_temperature
  if (lhs->highest_temperature != rhs->highest_temperature) {
    return false;
  }
  // lowest_temperature
  if (lhs->lowest_temperature != rhs->lowest_temperature) {
    return false;
  }
  // temperature
  if (lhs->temperature != rhs->temperature) {
    return false;
  }
  // pitch
  if (lhs->pitch != rhs->pitch) {
    return false;
  }
  // roll
  if (lhs->roll != rhs->roll) {
    return false;
  }
  // yaw
  if (lhs->yaw != rhs->yaw) {
    return false;
  }
  // wifi_snr
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->wifi_snr), &(rhs->wifi_snr)))
  {
    return false;
  }
  return true;
}

bool
tello_msg__msg__TelloStatus__copy(
  const tello_msg__msg__TelloStatus * input,
  tello_msg__msg__TelloStatus * output)
{
  if (!input || !output) {
    return false;
  }
  // acceleration
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->acceleration), &(output->acceleration)))
  {
    return false;
  }
  // speed
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->speed), &(output->speed)))
  {
    return false;
  }
  // barometer
  output->barometer = input->barometer;
  // distance_tof
  output->distance_tof = input->distance_tof;
  // battery
  output->battery = input->battery;
  // fligth_time
  output->fligth_time = input->fligth_time;
  // highest_temperature
  output->highest_temperature = input->highest_temperature;
  // lowest_temperature
  output->lowest_temperature = input->lowest_temperature;
  // temperature
  output->temperature = input->temperature;
  // pitch
  output->pitch = input->pitch;
  // roll
  output->roll = input->roll;
  // yaw
  output->yaw = input->yaw;
  // wifi_snr
  if (!rosidl_runtime_c__String__copy(
      &(input->wifi_snr), &(output->wifi_snr)))
  {
    return false;
  }
  return true;
}

tello_msg__msg__TelloStatus *
tello_msg__msg__TelloStatus__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tello_msg__msg__TelloStatus * msg = (tello_msg__msg__TelloStatus *)allocator.allocate(sizeof(tello_msg__msg__TelloStatus), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(tello_msg__msg__TelloStatus));
  bool success = tello_msg__msg__TelloStatus__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
tello_msg__msg__TelloStatus__destroy(tello_msg__msg__TelloStatus * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    tello_msg__msg__TelloStatus__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
tello_msg__msg__TelloStatus__Sequence__init(tello_msg__msg__TelloStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tello_msg__msg__TelloStatus * data = NULL;

  if (size) {
    data = (tello_msg__msg__TelloStatus *)allocator.zero_allocate(size, sizeof(tello_msg__msg__TelloStatus), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = tello_msg__msg__TelloStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        tello_msg__msg__TelloStatus__fini(&data[i - 1]);
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
tello_msg__msg__TelloStatus__Sequence__fini(tello_msg__msg__TelloStatus__Sequence * array)
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
      tello_msg__msg__TelloStatus__fini(&array->data[i]);
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

tello_msg__msg__TelloStatus__Sequence *
tello_msg__msg__TelloStatus__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tello_msg__msg__TelloStatus__Sequence * array = (tello_msg__msg__TelloStatus__Sequence *)allocator.allocate(sizeof(tello_msg__msg__TelloStatus__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = tello_msg__msg__TelloStatus__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
tello_msg__msg__TelloStatus__Sequence__destroy(tello_msg__msg__TelloStatus__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    tello_msg__msg__TelloStatus__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
tello_msg__msg__TelloStatus__Sequence__are_equal(const tello_msg__msg__TelloStatus__Sequence * lhs, const tello_msg__msg__TelloStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!tello_msg__msg__TelloStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
tello_msg__msg__TelloStatus__Sequence__copy(
  const tello_msg__msg__TelloStatus__Sequence * input,
  tello_msg__msg__TelloStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(tello_msg__msg__TelloStatus);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    tello_msg__msg__TelloStatus * data =
      (tello_msg__msg__TelloStatus *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!tello_msg__msg__TelloStatus__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          tello_msg__msg__TelloStatus__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!tello_msg__msg__TelloStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
