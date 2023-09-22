# generated from rosidl_generator_py/resource/_idl.py.em
# with input from tello_msg:msg/TelloStatus.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_TelloStatus(type):
    """Metaclass of message 'TelloStatus'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('tello_msg')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'tello_msg.msg.TelloStatus')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__tello_status
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__tello_status
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__tello_status
            cls._TYPE_SUPPORT = module.type_support_msg__msg__tello_status
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__tello_status

            from geometry_msgs.msg import Vector3
            if Vector3.__class__._TYPE_SUPPORT is None:
                Vector3.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class TelloStatus(metaclass=Metaclass_TelloStatus):
    """Message class 'TelloStatus'."""

    __slots__ = [
        '_acceleration',
        '_speed',
        '_barometer',
        '_distance_tof',
        '_battery',
        '_fligth_time',
        '_highest_temperature',
        '_lowest_temperature',
        '_temperature',
        '_pitch',
        '_roll',
        '_yaw',
        '_wifi_snr',
    ]

    _fields_and_field_types = {
        'acceleration': 'geometry_msgs/Vector3',
        'speed': 'geometry_msgs/Vector3',
        'barometer': 'int32',
        'distance_tof': 'int32',
        'battery': 'uint8',
        'fligth_time': 'int32',
        'highest_temperature': 'int32',
        'lowest_temperature': 'int32',
        'temperature': 'float',
        'pitch': 'int32',
        'roll': 'int32',
        'yaw': 'int32',
        'wifi_snr': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Vector3'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Vector3'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from geometry_msgs.msg import Vector3
        self.acceleration = kwargs.get('acceleration', Vector3())
        from geometry_msgs.msg import Vector3
        self.speed = kwargs.get('speed', Vector3())
        self.barometer = kwargs.get('barometer', int())
        self.distance_tof = kwargs.get('distance_tof', int())
        self.battery = kwargs.get('battery', int())
        self.fligth_time = kwargs.get('fligth_time', int())
        self.highest_temperature = kwargs.get('highest_temperature', int())
        self.lowest_temperature = kwargs.get('lowest_temperature', int())
        self.temperature = kwargs.get('temperature', float())
        self.pitch = kwargs.get('pitch', int())
        self.roll = kwargs.get('roll', int())
        self.yaw = kwargs.get('yaw', int())
        self.wifi_snr = kwargs.get('wifi_snr', str())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.acceleration != other.acceleration:
            return False
        if self.speed != other.speed:
            return False
        if self.barometer != other.barometer:
            return False
        if self.distance_tof != other.distance_tof:
            return False
        if self.battery != other.battery:
            return False
        if self.fligth_time != other.fligth_time:
            return False
        if self.highest_temperature != other.highest_temperature:
            return False
        if self.lowest_temperature != other.lowest_temperature:
            return False
        if self.temperature != other.temperature:
            return False
        if self.pitch != other.pitch:
            return False
        if self.roll != other.roll:
            return False
        if self.yaw != other.yaw:
            return False
        if self.wifi_snr != other.wifi_snr:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def acceleration(self):
        """Message field 'acceleration'."""
        return self._acceleration

    @acceleration.setter
    def acceleration(self, value):
        if __debug__:
            from geometry_msgs.msg import Vector3
            assert \
                isinstance(value, Vector3), \
                "The 'acceleration' field must be a sub message of type 'Vector3'"
        self._acceleration = value

    @builtins.property
    def speed(self):
        """Message field 'speed'."""
        return self._speed

    @speed.setter
    def speed(self, value):
        if __debug__:
            from geometry_msgs.msg import Vector3
            assert \
                isinstance(value, Vector3), \
                "The 'speed' field must be a sub message of type 'Vector3'"
        self._speed = value

    @builtins.property
    def barometer(self):
        """Message field 'barometer'."""
        return self._barometer

    @barometer.setter
    def barometer(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'barometer' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'barometer' field must be an integer in [-2147483648, 2147483647]"
        self._barometer = value

    @builtins.property
    def distance_tof(self):
        """Message field 'distance_tof'."""
        return self._distance_tof

    @distance_tof.setter
    def distance_tof(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'distance_tof' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'distance_tof' field must be an integer in [-2147483648, 2147483647]"
        self._distance_tof = value

    @builtins.property
    def battery(self):
        """Message field 'battery'."""
        return self._battery

    @battery.setter
    def battery(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'battery' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'battery' field must be an unsigned integer in [0, 255]"
        self._battery = value

    @builtins.property
    def fligth_time(self):
        """Message field 'fligth_time'."""
        return self._fligth_time

    @fligth_time.setter
    def fligth_time(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'fligth_time' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'fligth_time' field must be an integer in [-2147483648, 2147483647]"
        self._fligth_time = value

    @builtins.property
    def highest_temperature(self):
        """Message field 'highest_temperature'."""
        return self._highest_temperature

    @highest_temperature.setter
    def highest_temperature(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'highest_temperature' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'highest_temperature' field must be an integer in [-2147483648, 2147483647]"
        self._highest_temperature = value

    @builtins.property
    def lowest_temperature(self):
        """Message field 'lowest_temperature'."""
        return self._lowest_temperature

    @lowest_temperature.setter
    def lowest_temperature(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'lowest_temperature' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'lowest_temperature' field must be an integer in [-2147483648, 2147483647]"
        self._lowest_temperature = value

    @builtins.property
    def temperature(self):
        """Message field 'temperature'."""
        return self._temperature

    @temperature.setter
    def temperature(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'temperature' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'temperature' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._temperature = value

    @builtins.property
    def pitch(self):
        """Message field 'pitch'."""
        return self._pitch

    @pitch.setter
    def pitch(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'pitch' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'pitch' field must be an integer in [-2147483648, 2147483647]"
        self._pitch = value

    @builtins.property
    def roll(self):
        """Message field 'roll'."""
        return self._roll

    @roll.setter
    def roll(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'roll' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'roll' field must be an integer in [-2147483648, 2147483647]"
        self._roll = value

    @builtins.property
    def yaw(self):
        """Message field 'yaw'."""
        return self._yaw

    @yaw.setter
    def yaw(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'yaw' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'yaw' field must be an integer in [-2147483648, 2147483647]"
        self._yaw = value

    @builtins.property
    def wifi_snr(self):
        """Message field 'wifi_snr'."""
        return self._wifi_snr

    @wifi_snr.setter
    def wifi_snr(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'wifi_snr' field must be of type 'str'"
        self._wifi_snr = value
