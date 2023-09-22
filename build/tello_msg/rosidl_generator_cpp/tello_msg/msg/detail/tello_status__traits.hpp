// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from tello_msg:msg/TelloStatus.idl
// generated code does not contain a copyright notice

#ifndef TELLO_MSG__MSG__DETAIL__TELLO_STATUS__TRAITS_HPP_
#define TELLO_MSG__MSG__DETAIL__TELLO_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "tello_msg/msg/detail/tello_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'acceleration'
// Member 'speed'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"

namespace tello_msg
{

namespace msg
{

inline void to_flow_style_yaml(
  const TelloStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: acceleration
  {
    out << "acceleration: ";
    to_flow_style_yaml(msg.acceleration, out);
    out << ", ";
  }

  // member: speed
  {
    out << "speed: ";
    to_flow_style_yaml(msg.speed, out);
    out << ", ";
  }

  // member: barometer
  {
    out << "barometer: ";
    rosidl_generator_traits::value_to_yaml(msg.barometer, out);
    out << ", ";
  }

  // member: distance_tof
  {
    out << "distance_tof: ";
    rosidl_generator_traits::value_to_yaml(msg.distance_tof, out);
    out << ", ";
  }

  // member: battery
  {
    out << "battery: ";
    rosidl_generator_traits::value_to_yaml(msg.battery, out);
    out << ", ";
  }

  // member: fligth_time
  {
    out << "fligth_time: ";
    rosidl_generator_traits::value_to_yaml(msg.fligth_time, out);
    out << ", ";
  }

  // member: highest_temperature
  {
    out << "highest_temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.highest_temperature, out);
    out << ", ";
  }

  // member: lowest_temperature
  {
    out << "lowest_temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.lowest_temperature, out);
    out << ", ";
  }

  // member: temperature
  {
    out << "temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.temperature, out);
    out << ", ";
  }

  // member: pitch
  {
    out << "pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch, out);
    out << ", ";
  }

  // member: roll
  {
    out << "roll: ";
    rosidl_generator_traits::value_to_yaml(msg.roll, out);
    out << ", ";
  }

  // member: yaw
  {
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
    out << ", ";
  }

  // member: wifi_snr
  {
    out << "wifi_snr: ";
    rosidl_generator_traits::value_to_yaml(msg.wifi_snr, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TelloStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: acceleration
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "acceleration:\n";
    to_block_style_yaml(msg.acceleration, out, indentation + 2);
  }

  // member: speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "speed:\n";
    to_block_style_yaml(msg.speed, out, indentation + 2);
  }

  // member: barometer
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "barometer: ";
    rosidl_generator_traits::value_to_yaml(msg.barometer, out);
    out << "\n";
  }

  // member: distance_tof
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "distance_tof: ";
    rosidl_generator_traits::value_to_yaml(msg.distance_tof, out);
    out << "\n";
  }

  // member: battery
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "battery: ";
    rosidl_generator_traits::value_to_yaml(msg.battery, out);
    out << "\n";
  }

  // member: fligth_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fligth_time: ";
    rosidl_generator_traits::value_to_yaml(msg.fligth_time, out);
    out << "\n";
  }

  // member: highest_temperature
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "highest_temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.highest_temperature, out);
    out << "\n";
  }

  // member: lowest_temperature
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "lowest_temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.lowest_temperature, out);
    out << "\n";
  }

  // member: temperature
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.temperature, out);
    out << "\n";
  }

  // member: pitch
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch, out);
    out << "\n";
  }

  // member: roll
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "roll: ";
    rosidl_generator_traits::value_to_yaml(msg.roll, out);
    out << "\n";
  }

  // member: yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
    out << "\n";
  }

  // member: wifi_snr
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "wifi_snr: ";
    rosidl_generator_traits::value_to_yaml(msg.wifi_snr, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TelloStatus & msg, bool use_flow_style = false)
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
  const tello_msg::msg::TelloStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  tello_msg::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use tello_msg::msg::to_yaml() instead")]]
inline std::string to_yaml(const tello_msg::msg::TelloStatus & msg)
{
  return tello_msg::msg::to_yaml(msg);
}

template<>
inline const char * data_type<tello_msg::msg::TelloStatus>()
{
  return "tello_msg::msg::TelloStatus";
}

template<>
inline const char * name<tello_msg::msg::TelloStatus>()
{
  return "tello_msg/msg/TelloStatus";
}

template<>
struct has_fixed_size<tello_msg::msg::TelloStatus>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<tello_msg::msg::TelloStatus>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<tello_msg::msg::TelloStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TELLO_MSG__MSG__DETAIL__TELLO_STATUS__TRAITS_HPP_
