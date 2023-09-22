// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from tello_msg:msg/TelloID.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "tello_msg/msg/detail/tello_id__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace tello_msg
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void TelloID_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) tello_msg::msg::TelloID(_init);
}

void TelloID_fini_function(void * message_memory)
{
  auto typed_message = static_cast<tello_msg::msg::TelloID *>(message_memory);
  typed_message->~TelloID();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember TelloID_message_member_array[2] = {
  {
    "sdk_version",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tello_msg::msg::TelloID, sdk_version),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "serial_number",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tello_msg::msg::TelloID, serial_number),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers TelloID_message_members = {
  "tello_msg::msg",  // message namespace
  "TelloID",  // message name
  2,  // number of fields
  sizeof(tello_msg::msg::TelloID),
  TelloID_message_member_array,  // message members
  TelloID_init_function,  // function to initialize message memory (memory has to be allocated)
  TelloID_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t TelloID_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &TelloID_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace tello_msg


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<tello_msg::msg::TelloID>()
{
  return &::tello_msg::msg::rosidl_typesupport_introspection_cpp::TelloID_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, tello_msg, msg, TelloID)() {
  return &::tello_msg::msg::rosidl_typesupport_introspection_cpp::TelloID_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
