// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from tello_msg:msg/TelloID.idl
// generated code does not contain a copyright notice

#ifndef TELLO_MSG__MSG__DETAIL__TELLO_ID__STRUCT_HPP_
#define TELLO_MSG__MSG__DETAIL__TELLO_ID__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__tello_msg__msg__TelloID __attribute__((deprecated))
#else
# define DEPRECATED__tello_msg__msg__TelloID __declspec(deprecated)
#endif

namespace tello_msg
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TelloID_
{
  using Type = TelloID_<ContainerAllocator>;

  explicit TelloID_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->sdk_version = "";
      this->serial_number = "";
    }
  }

  explicit TelloID_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : sdk_version(_alloc),
    serial_number(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->sdk_version = "";
      this->serial_number = "";
    }
  }

  // field types and members
  using _sdk_version_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _sdk_version_type sdk_version;
  using _serial_number_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _serial_number_type serial_number;

  // setters for named parameter idiom
  Type & set__sdk_version(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->sdk_version = _arg;
    return *this;
  }
  Type & set__serial_number(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->serial_number = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    tello_msg::msg::TelloID_<ContainerAllocator> *;
  using ConstRawPtr =
    const tello_msg::msg::TelloID_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<tello_msg::msg::TelloID_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<tello_msg::msg::TelloID_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      tello_msg::msg::TelloID_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<tello_msg::msg::TelloID_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      tello_msg::msg::TelloID_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<tello_msg::msg::TelloID_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<tello_msg::msg::TelloID_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<tello_msg::msg::TelloID_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__tello_msg__msg__TelloID
    std::shared_ptr<tello_msg::msg::TelloID_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__tello_msg__msg__TelloID
    std::shared_ptr<tello_msg::msg::TelloID_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TelloID_ & other) const
  {
    if (this->sdk_version != other.sdk_version) {
      return false;
    }
    if (this->serial_number != other.serial_number) {
      return false;
    }
    return true;
  }
  bool operator!=(const TelloID_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TelloID_

// alias to use template instance with default allocator
using TelloID =
  tello_msg::msg::TelloID_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace tello_msg

#endif  // TELLO_MSG__MSG__DETAIL__TELLO_ID__STRUCT_HPP_
