// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from tello_msg:msg/TelloStatus.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "tello_msg/msg/detail/tello_status__struct.h"
#include "tello_msg/msg/detail/tello_status__functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool geometry_msgs__msg__vector3__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * geometry_msgs__msg__vector3__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool geometry_msgs__msg__vector3__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * geometry_msgs__msg__vector3__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool tello_msg__msg__tello_status__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[40];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("tello_msg.msg._tello_status.TelloStatus", full_classname_dest, 39) == 0);
  }
  tello_msg__msg__TelloStatus * ros_message = _ros_message;
  {  // acceleration
    PyObject * field = PyObject_GetAttrString(_pymsg, "acceleration");
    if (!field) {
      return false;
    }
    if (!geometry_msgs__msg__vector3__convert_from_py(field, &ros_message->acceleration)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // speed
    PyObject * field = PyObject_GetAttrString(_pymsg, "speed");
    if (!field) {
      return false;
    }
    if (!geometry_msgs__msg__vector3__convert_from_py(field, &ros_message->speed)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // barometer
    PyObject * field = PyObject_GetAttrString(_pymsg, "barometer");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->barometer = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // distance_tof
    PyObject * field = PyObject_GetAttrString(_pymsg, "distance_tof");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->distance_tof = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // battery
    PyObject * field = PyObject_GetAttrString(_pymsg, "battery");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->battery = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // fligth_time
    PyObject * field = PyObject_GetAttrString(_pymsg, "fligth_time");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->fligth_time = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // highest_temperature
    PyObject * field = PyObject_GetAttrString(_pymsg, "highest_temperature");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->highest_temperature = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // lowest_temperature
    PyObject * field = PyObject_GetAttrString(_pymsg, "lowest_temperature");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->lowest_temperature = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // temperature
    PyObject * field = PyObject_GetAttrString(_pymsg, "temperature");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->temperature = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // pitch
    PyObject * field = PyObject_GetAttrString(_pymsg, "pitch");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->pitch = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // roll
    PyObject * field = PyObject_GetAttrString(_pymsg, "roll");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->roll = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // yaw
    PyObject * field = PyObject_GetAttrString(_pymsg, "yaw");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->yaw = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // wifi_snr
    PyObject * field = PyObject_GetAttrString(_pymsg, "wifi_snr");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->wifi_snr, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * tello_msg__msg__tello_status__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of TelloStatus */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("tello_msg.msg._tello_status");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "TelloStatus");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  tello_msg__msg__TelloStatus * ros_message = (tello_msg__msg__TelloStatus *)raw_ros_message;
  {  // acceleration
    PyObject * field = NULL;
    field = geometry_msgs__msg__vector3__convert_to_py(&ros_message->acceleration);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "acceleration", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // speed
    PyObject * field = NULL;
    field = geometry_msgs__msg__vector3__convert_to_py(&ros_message->speed);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "speed", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // barometer
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->barometer);
    {
      int rc = PyObject_SetAttrString(_pymessage, "barometer", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // distance_tof
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->distance_tof);
    {
      int rc = PyObject_SetAttrString(_pymessage, "distance_tof", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // battery
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->battery);
    {
      int rc = PyObject_SetAttrString(_pymessage, "battery", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // fligth_time
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->fligth_time);
    {
      int rc = PyObject_SetAttrString(_pymessage, "fligth_time", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // highest_temperature
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->highest_temperature);
    {
      int rc = PyObject_SetAttrString(_pymessage, "highest_temperature", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // lowest_temperature
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->lowest_temperature);
    {
      int rc = PyObject_SetAttrString(_pymessage, "lowest_temperature", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // temperature
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->temperature);
    {
      int rc = PyObject_SetAttrString(_pymessage, "temperature", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // pitch
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->pitch);
    {
      int rc = PyObject_SetAttrString(_pymessage, "pitch", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // roll
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->roll);
    {
      int rc = PyObject_SetAttrString(_pymessage, "roll", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // yaw
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->yaw);
    {
      int rc = PyObject_SetAttrString(_pymessage, "yaw", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // wifi_snr
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->wifi_snr.data,
      strlen(ros_message->wifi_snr.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "wifi_snr", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
