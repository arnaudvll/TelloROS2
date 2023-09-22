// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from tello_msg:msg/TelloWifiConfig.idl
// generated code does not contain a copyright notice

#ifndef TELLO_MSG__MSG__DETAIL__TELLO_WIFI_CONFIG__TRAITS_HPP_
#define TELLO_MSG__MSG__DETAIL__TELLO_WIFI_CONFIG__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "tello_msg/msg/detail/tello_wifi_config__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace tello_msg
{

namespace msg
{

inline void to_flow_style_yaml(
  const TelloWifiConfig & msg,
  std::ostream & out)
{
  out << "{";
  // member: ssid
  {
    out << "ssid: ";
    rosidl_generator_traits::value_to_yaml(msg.ssid, out);
    out << ", ";
  }

  // member: password
  {
    out << "password: ";
    rosidl_generator_traits::value_to_yaml(msg.password, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TelloWifiConfig & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: ssid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ssid: ";
    rosidl_generator_traits::value_to_yaml(msg.ssid, out);
    out << "\n";
  }

  // member: password
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "password: ";
    rosidl_generator_traits::value_to_yaml(msg.password, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TelloWifiConfig & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace tello_msg

namespace rosidl_generator_traits
{

[[deprecated("use tello_msg::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const tello_msg::msg::TelloWifiConfig & msg,
  std::ostream & out, size_t indentation = 0)
{
  tello_msg::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use tello_msg::msg::to_yaml() instead")]]
inline std::string to_yaml(const tello_msg::msg::TelloWifiConfig & msg)
{
  return tello_msg::msg::to_yaml(msg);
}

template<>
inline const char * data_type<tello_msg::msg::TelloWifiConfig>()
{
  return "tello_msg::msg::TelloWifiConfig";
}

template<>
inline const char * name<tello_msg::msg::TelloWifiConfig>()
{
  return "tello_msg/msg/TelloWifiConfig";
}

template<>
struct has_fixed_size<tello_msg::msg::TelloWifiConfig>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<tello_msg::msg::TelloWifiConfig>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<tello_msg::msg::TelloWifiConfig>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TELLO_MSG__MSG__DETAIL__TELLO_WIFI_CONFIG__TRAITS_HPP_
