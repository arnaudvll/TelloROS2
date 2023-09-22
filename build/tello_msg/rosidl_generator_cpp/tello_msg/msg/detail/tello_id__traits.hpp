// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from tello_msg:msg/TelloID.idl
// generated code does not contain a copyright notice

#ifndef TELLO_MSG__MSG__DETAIL__TELLO_ID__TRAITS_HPP_
#define TELLO_MSG__MSG__DETAIL__TELLO_ID__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "tello_msg/msg/detail/tello_id__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace tello_msg
{

namespace msg
{

inline void to_flow_style_yaml(
  const TelloID & msg,
  std::ostream & out)
{
  out << "{";
  // member: sdk_version
  {
    out << "sdk_version: ";
    rosidl_generator_traits::value_to_yaml(msg.sdk_version, out);
    out << ", ";
  }

  // member: serial_number
  {
    out << "serial_number: ";
    rosidl_generator_traits::value_to_yaml(msg.serial_number, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TelloID & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: sdk_version
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sdk_version: ";
    rosidl_generator_traits::value_to_yaml(msg.sdk_version, out);
    out << "\n";
  }

  // member: serial_number
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "serial_number: ";
    rosidl_generator_traits::value_to_yaml(msg.serial_number, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TelloID & msg, bool use_flow_style = false)
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
  const tello_msg::msg::TelloID & msg,
  std::ostream & out, size_t indentation = 0)
{
  tello_msg::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use tello_msg::msg::to_yaml() instead")]]
inline std::string to_yaml(const tello_msg::msg::TelloID & msg)
{
  return tello_msg::msg::to_yaml(msg);
}

template<>
inline const char * data_type<tello_msg::msg::TelloID>()
{
  return "tello_msg::msg::TelloID";
}

template<>
inline const char * name<tello_msg::msg::TelloID>()
{
  return "tello_msg/msg/TelloID";
}

template<>
struct has_fixed_size<tello_msg::msg::TelloID>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<tello_msg::msg::TelloID>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<tello_msg::msg::TelloID>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TELLO_MSG__MSG__DETAIL__TELLO_ID__TRAITS_HPP_
