// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from tello_msg:msg/TelloID.idl
// generated code does not contain a copyright notice

#ifndef TELLO_MSG__MSG__DETAIL__TELLO_ID__BUILDER_HPP_
#define TELLO_MSG__MSG__DETAIL__TELLO_ID__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "tello_msg/msg/detail/tello_id__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace tello_msg
{

namespace msg
{

namespace builder
{

class Init_TelloID_serial_number
{
public:
  explicit Init_TelloID_serial_number(::tello_msg::msg::TelloID & msg)
  : msg_(msg)
  {}
  ::tello_msg::msg::TelloID serial_number(::tello_msg::msg::TelloID::_serial_number_type arg)
  {
    msg_.serial_number = std::move(arg);
    return std::move(msg_);
  }

private:
  ::tello_msg::msg::TelloID msg_;
};

class Init_TelloID_sdk_version
{
public:
  Init_TelloID_sdk_version()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TelloID_serial_number sdk_version(::tello_msg::msg::TelloID::_sdk_version_type arg)
  {
    msg_.sdk_version = std::move(arg);
    return Init_TelloID_serial_number(msg_);
  }

private:
  ::tello_msg::msg::TelloID msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::tello_msg::msg::TelloID>()
{
  return tello_msg::msg::builder::Init_TelloID_sdk_version();
}

}  // namespace tello_msg

#endif  // TELLO_MSG__MSG__DETAIL__TELLO_ID__BUILDER_HPP_
