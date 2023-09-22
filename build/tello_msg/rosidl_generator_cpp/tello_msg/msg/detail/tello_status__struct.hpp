// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from tello_msg:msg/TelloStatus.idl
// generated code does not contain a copyright notice

#ifndef TELLO_MSG__MSG__DETAIL__TELLO_STATUS__STRUCT_HPP_
#define TELLO_MSG__MSG__DETAIL__TELLO_STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'acceleration'
// Member 'speed'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__tello_msg__msg__TelloStatus __attribute__((deprecated))
#else
# define DEPRECATED__tello_msg__msg__TelloStatus __declspec(deprecated)
#endif

namespace tello_msg
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TelloStatus_
{
  using Type = TelloStatus_<ContainerAllocator>;

  explicit TelloStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : acceleration(_init),
    speed(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->barometer = 0l;
      this->distance_tof = 0l;
      this->battery = 0;
      this->fligth_time = 0l;
      this->highest_temperature = 0l;
      this->lowest_temperature = 0l;
      this->temperature = 0.0f;
      this->pitch = 0l;
      this->roll = 0l;
      this->yaw = 0l;
      this->wifi_snr = "";
    }
  }

  explicit TelloStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : acceleration(_alloc, _init),
    speed(_alloc, _init),
    wifi_snr(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->barometer = 0l;
      this->distance_tof = 0l;
      this->battery = 0;
      this->fligth_time = 0l;
      this->highest_temperature = 0l;
      this->lowest_temperature = 0l;
      this->temperature = 0.0f;
      this->pitch = 0l;
      this->roll = 0l;
      this->yaw = 0l;
      this->wifi_snr = "";
    }
  }

  // field types and members
  using _acceleration_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _acceleration_type acceleration;
  using _speed_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _speed_type speed;
  using _barometer_type =
    int32_t;
  _barometer_type barometer;
  using _distance_tof_type =
    int32_t;
  _distance_tof_type distance_tof;
  using _battery_type =
    uint8_t;
  _battery_type battery;
  using _fligth_time_type =
    int32_t;
  _fligth_time_type fligth_time;
  using _highest_temperature_type =
    int32_t;
  _highest_temperature_type highest_temperature;
  using _lowest_temperature_type =
    int32_t;
  _lowest_temperature_type lowest_temperature;
  using _temperature_type =
    float;
  _temperature_type temperature;
  using _pitch_type =
    int32_t;
  _pitch_type pitch;
  using _roll_type =
    int32_t;
  _roll_type roll;
  using _yaw_type =
    int32_t;
  _yaw_type yaw;
  using _wifi_snr_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _wifi_snr_type wifi_snr;

  // setters for named parameter idiom
  Type & set__acceleration(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->acceleration = _arg;
    return *this;
  }
  Type & set__speed(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->speed = _arg;
    return *this;
  }
  Type & set__barometer(
    const int32_t & _arg)
  {
    this->barometer = _arg;
    return *this;
  }
  Type & set__distance_tof(
    const int32_t & _arg)
  {
    this->distance_tof = _arg;
    return *this;
  }
  Type & set__battery(
    const uint8_t & _arg)
  {
    this->battery = _arg;
    return *this;
  }
  Type & set__fligth_time(
    const int32_t & _arg)
  {
    this->fligth_time = _arg;
    return *this;
  }
  Type & set__highest_temperature(
    const int32_t & _arg)
  {
    this->highest_temperature = _arg;
    return *this;
  }
  Type & set__lowest_temperature(
    const int32_t & _arg)
  {
    this->lowest_temperature = _arg;
    return *this;
  }
  Type & set__temperature(
    const float & _arg)
  {
    this->temperature = _arg;
    return *this;
  }
  Type & set__pitch(
    const int32_t & _arg)
  {
    this->pitch = _arg;
    return *this;
  }
  Type & set__roll(
    const int32_t & _arg)
  {
    this->roll = _arg;
    return *this;
  }
  Type & set__yaw(
    const int32_t & _arg)
  {
    this->yaw = _arg;
    return *this;
  }
  Type & set__wifi_snr(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->wifi_snr = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    tello_msg::msg::TelloStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const tello_msg::msg::TelloStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<tello_msg::msg::TelloStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<tello_msg::msg::TelloStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      tello_msg::msg::TelloStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<tello_msg::msg::TelloStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      tello_msg::msg::TelloStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<tello_msg::msg::TelloStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<tello_msg::msg::TelloStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<tello_msg::msg::TelloStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__tello_msg__msg__TelloStatus
    std::shared_ptr<tello_msg::msg::TelloStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__tello_msg__msg__TelloStatus
    std::shared_ptr<tello_msg::msg::TelloStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TelloStatus_ & other) const
  {
    if (this->acceleration != other.acceleration) {
      return false;
    }
    if (this->speed != other.speed) {
      return false;
    }
    if (this->barometer != other.barometer) {
      return false;
    }
    if (this->distance_tof != other.distance_tof) {
      return false;
    }
    if (this->battery != other.battery) {
      return false;
    }
    if (this->fligth_time != other.fligth_time) {
      return false;
    }
    if (this->highest_temperature != other.highest_temperature) {
      return false;
    }
    if (this->lowest_temperature != other.lowest_temperature) {
      return false;
    }
    if (this->temperature != other.temperature) {
      return false;
    }
    if (this->pitch != other.pitch) {
      return false;
    }
    if (this->roll != other.roll) {
      return false;
    }
    if (this->yaw != other.yaw) {
      return false;
    }
    if (this->wifi_snr != other.wifi_snr) {
      return false;
    }
    return true;
  }
  bool operator!=(const TelloStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TelloStatus_

// alias to use template instance with default allocator
using TelloStatus =
  tello_msg::msg::TelloStatus_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace tello_msg

#endif  // TELLO_MSG__MSG__DETAIL__TELLO_STATUS__STRUCT_HPP_
