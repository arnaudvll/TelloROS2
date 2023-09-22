// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from tello_msg:msg/TelloStatus.idl
// generated code does not contain a copyright notice

#ifndef TELLO_MSG__MSG__DETAIL__TELLO_STATUS__BUILDER_HPP_
#define TELLO_MSG__MSG__DETAIL__TELLO_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "tello_msg/msg/detail/tello_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace tello_msg
{

namespace msg
{

namespace builder
{

class Init_TelloStatus_wifi_snr
{
public:
  explicit Init_TelloStatus_wifi_snr(::tello_msg::msg::TelloStatus & msg)
  : msg_(msg)
  {}
  ::tello_msg::msg::TelloStatus wifi_snr(::tello_msg::msg::TelloStatus::_wifi_snr_type arg)
  {
    msg_.wifi_snr = std::move(arg);
    return std::move(msg_);
  }

private:
  ::tello_msg::msg::TelloStatus msg_;
};

class Init_TelloStatus_yaw
{
public:
  explicit Init_TelloStatus_yaw(::tello_msg::msg::TelloStatus & msg)
  : msg_(msg)
  {}
  Init_TelloStatus_wifi_snr yaw(::tello_msg::msg::TelloStatus::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return Init_TelloStatus_wifi_snr(msg_);
  }

private:
  ::tello_msg::msg::TelloStatus msg_;
};

class Init_TelloStatus_roll
{
public:
  explicit Init_TelloStatus_roll(::tello_msg::msg::TelloStatus & msg)
  : msg_(msg)
  {}
  Init_TelloStatus_yaw roll(::tello_msg::msg::TelloStatus::_roll_type arg)
  {
    msg_.roll = std::move(arg);
    return Init_TelloStatus_yaw(msg_);
  }

private:
  ::tello_msg::msg::TelloStatus msg_;
};

class Init_TelloStatus_pitch
{
public:
  explicit Init_TelloStatus_pitch(::tello_msg::msg::TelloStatus & msg)
  : msg_(msg)
  {}
  Init_TelloStatus_roll pitch(::tello_msg::msg::TelloStatus::_pitch_type arg)
  {
    msg_.pitch = std::move(arg);
    return Init_TelloStatus_roll(msg_);
  }

private:
  ::tello_msg::msg::TelloStatus msg_;
};

class Init_TelloStatus_temperature
{
public:
  explicit Init_TelloStatus_temperature(::tello_msg::msg::TelloStatus & msg)
  : msg_(msg)
  {}
  Init_TelloStatus_pitch temperature(::tello_msg::msg::TelloStatus::_temperature_type arg)
  {
    msg_.temperature = std::move(arg);
    return Init_TelloStatus_pitch(msg_);
  }

private:
  ::tello_msg::msg::TelloStatus msg_;
};

class Init_TelloStatus_lowest_temperature
{
public:
  explicit Init_TelloStatus_lowest_temperature(::tello_msg::msg::TelloStatus & msg)
  : msg_(msg)
  {}
  Init_TelloStatus_temperature lowest_temperature(::tello_msg::msg::TelloStatus::_lowest_temperature_type arg)
  {
    msg_.lowest_temperature = std::move(arg);
    return Init_TelloStatus_temperature(msg_);
  }

private:
  ::tello_msg::msg::TelloStatus msg_;
};

class Init_TelloStatus_highest_temperature
{
public:
  explicit Init_TelloStatus_highest_temperature(::tello_msg::msg::TelloStatus & msg)
  : msg_(msg)
  {}
  Init_TelloStatus_lowest_temperature highest_temperature(::tello_msg::msg::TelloStatus::_highest_temperature_type arg)
  {
    msg_.highest_temperature = std::move(arg);
    return Init_TelloStatus_lowest_temperature(msg_);
  }

private:
  ::tello_msg::msg::TelloStatus msg_;
};

class Init_TelloStatus_fligth_time
{
public:
  explicit Init_TelloStatus_fligth_time(::tello_msg::msg::TelloStatus & msg)
  : msg_(msg)
  {}
  Init_TelloStatus_highest_temperature fligth_time(::tello_msg::msg::TelloStatus::_fligth_time_type arg)
  {
    msg_.fligth_time = std::move(arg);
    return Init_TelloStatus_highest_temperature(msg_);
  }

private:
  ::tello_msg::msg::TelloStatus msg_;
};

class Init_TelloStatus_battery
{
public:
  explicit Init_TelloStatus_battery(::tello_msg::msg::TelloStatus & msg)
  : msg_(msg)
  {}
  Init_TelloStatus_fligth_time battery(::tello_msg::msg::TelloStatus::_battery_type arg)
  {
    msg_.battery = std::move(arg);
    return Init_TelloStatus_fligth_time(msg_);
  }

private:
  ::tello_msg::msg::TelloStatus msg_;
};

class Init_TelloStatus_distance_tof
{
public:
  explicit Init_TelloStatus_distance_tof(::tello_msg::msg::TelloStatus & msg)
  : msg_(msg)
  {}
  Init_TelloStatus_battery distance_tof(::tello_msg::msg::TelloStatus::_distance_tof_type arg)
  {
    msg_.distance_tof = std::move(arg);
    return Init_TelloStatus_battery(msg_);
  }

private:
  ::tello_msg::msg::TelloStatus msg_;
};

class Init_TelloStatus_barometer
{
public:
  explicit Init_TelloStatus_barometer(::tello_msg::msg::TelloStatus & msg)
  : msg_(msg)
  {}
  Init_TelloStatus_distance_tof barometer(::tello_msg::msg::TelloStatus::_barometer_type arg)
  {
    msg_.barometer = std::move(arg);
    return Init_TelloStatus_distance_tof(msg_);
  }

private:
  ::tello_msg::msg::TelloStatus msg_;
};

class Init_TelloStatus_speed
{
public:
  explicit Init_TelloStatus_speed(::tello_msg::msg::TelloStatus & msg)
  : msg_(msg)
  {}
  Init_TelloStatus_barometer speed(::tello_msg::msg::TelloStatus::_speed_type arg)
  {
    msg_.speed = std::move(arg);
    return Init_TelloStatus_barometer(msg_);
  }

private:
  ::tello_msg::msg::TelloStatus msg_;
};

class Init_TelloStatus_acceleration
{
public:
  Init_TelloStatus_acceleration()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TelloStatus_speed acceleration(::tello_msg::msg::TelloStatus::_acceleration_type arg)
  {
    msg_.acceleration = std::move(arg);
    return Init_TelloStatus_speed(msg_);
  }

private:
  ::tello_msg::msg::TelloStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::tello_msg::msg::TelloStatus>()
{
  return tello_msg::msg::builder::Init_TelloStatus_acceleration();
}

}  // namespace tello_msg

#endif  // TELLO_MSG__MSG__DETAIL__TELLO_STATUS__BUILDER_HPP_
