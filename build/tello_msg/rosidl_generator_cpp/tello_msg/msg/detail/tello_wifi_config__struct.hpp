// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from tello_msg:msg/TelloWifiConfig.idl
// generated code does not contain a copyright notice

#ifndef TELLO_MSG__MSG__DETAIL__TELLO_WIFI_CONFIG__STRUCT_HPP_
#define TELLO_MSG__MSG__DETAIL__TELLO_WIFI_CONFIG__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__tello_msg__msg__TelloWifiConfig __attribute__((deprecated))
#else
# define DEPRECATED__tello_msg__msg__TelloWifiConfig __declspec(deprecated)
#endif

namespace tello_msg
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TelloWifiConfig_
{
  using Type = TelloWifiConfig_<ContainerAllocator>;

  explicit TelloWifiConfig_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->ssid = "";
      this->password = "";
    }
  }

  explicit TelloWifiConfig_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : ssid(_alloc),
    password(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->ssid = "";
      this->password = "";
    }
  }

  // field types and members
  using _ssid_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _ssid_type ssid;
  using _password_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _password_type password;

  // setters for named parameter idiom
  Type & set__ssid(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->ssid = _arg;
    return *this;
  }
  Type & set__password(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->password = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    tello_msg::msg::TelloWifiConfig_<ContainerAllocator> *;
  using ConstRawPtr =
    const tello_msg::msg::TelloWifiConfig_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<tello_msg::msg::TelloWifiConfig_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<tello_msg::msg::TelloWifiConfig_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      tello_msg::msg::TelloWifiConfig_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<tello_msg::msg::TelloWifiConfig_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      tello_msg::msg::TelloWifiConfig_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<tello_msg::msg::TelloWifiConfig_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<tello_msg::msg::TelloWifiConfig_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<tello_msg::msg::TelloWifiConfig_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__tello_msg__msg__TelloWifiConfig
    std::shared_ptr<tello_msg::msg::TelloWifiConfig_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__tello_msg__msg__TelloWifiConfig
    std::shared_ptr<tello_msg::msg::TelloWifiConfig_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TelloWifiConfig_ & other) const
  {
    if (this->ssid != other.ssid) {
      return false;
    }
    if (this->password != other.password) {
      return false;
    }
    return true;
  }
  bool operator!=(const TelloWifiConfig_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TelloWifiConfig_

// alias to use template instance with default allocator
using TelloWifiConfig =
  tello_msg::msg::TelloWifiConfig_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace tello_msg

#endif  // TELLO_MSG__MSG__DETAIL__TELLO_WIFI_CONFIG__STRUCT_HPP_
