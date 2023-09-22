// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from tello_msg:msg/TelloWifiConfig.idl
// generated code does not contain a copyright notice

#ifndef TELLO_MSG__MSG__DETAIL__TELLO_WIFI_CONFIG__BUILDER_HPP_
#define TELLO_MSG__MSG__DETAIL__TELLO_WIFI_CONFIG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "tello_msg/msg/detail/tello_wifi_config__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace tello_msg
{

namespace msg
{

namespace builder
{

class Init_TelloWifiConfig_password
{
public:
  explicit Init_TelloWifiConfig_password(::tello_msg::msg::TelloWifiConfig & msg)
  : msg_(msg)
  {}
  ::tello_msg::msg::TelloWifiConfig password(::tello_msg::msg::TelloWifiConfig::_password_type arg)
  {
    msg_.password = std::move(arg);
    return std::move(msg_);
  }

private:
  ::tello_msg::msg::TelloWifiConfig msg_;
};

class Init_TelloWifiConfig_ssid
{
public:
  Init_TelloWifiConfig_ssid()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TelloWifiConfig_password ssid(::tello_msg::msg::TelloWifiConfig::_ssid_type arg)
  {
    msg_.ssid = std::move(arg);
    return Init_TelloWifiConfig_password(msg_);
  }

private:
  ::tello_msg::msg::TelloWifiConfig msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::tello_msg::msg::TelloWifiConfig>()
{
  return tello_msg::msg::builder::Init_TelloWifiConfig_ssid();
}

}  // namespace tello_msg

#endif  // TELLO_MSG__MSG__DETAIL__TELLO_WIFI_CONFIG__BUILDER_HPP_
