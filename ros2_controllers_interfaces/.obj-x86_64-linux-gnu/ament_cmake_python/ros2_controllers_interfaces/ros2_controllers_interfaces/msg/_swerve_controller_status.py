# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ros2_controllers_interfaces:msg/SwerveControllerStatus.idl
# generated code does not contain a copyright notice

# This is being done at the module level and not on the instance level to avoid looking
# for the same variable multiple times on each instance. This variable is not supposed to
# change during runtime so it makes sense to only look for it once.
from os import getenv

ros_python_check_fields = getenv('ROS_PYTHON_CHECK_FIELDS', default='')


# Import statements for member types

# Member 'wheel_drive_position'
# Member 'wheel_drive_velocity'
# Member 'wheel_steer_position'
# Member 'wheel_icr'
# Member 'wheel_steering_angle_cmd'
# Member 'wheel_drive_velocity_cmd'
import array  # noqa: E402, I100

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_SwerveControllerStatus(type):
    """Metaclass of message 'SwerveControllerStatus'."""

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
            module = import_type_support('ros2_controllers_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'ros2_controllers_interfaces.msg.SwerveControllerStatus')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__swerve_controller_status
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__swerve_controller_status
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__swerve_controller_status
            cls._TYPE_SUPPORT = module.type_support_msg__msg__swerve_controller_status
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__swerve_controller_status

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SwerveControllerStatus(metaclass=Metaclass_SwerveControllerStatus):
    """Message class 'SwerveControllerStatus'."""

    __slots__ = [
        '_header',
        '_wheel_drive_position',
        '_wheel_drive_velocity',
        '_wheel_steer_position',
        '_wheel_icr',
        '_wheel_steering_angle_cmd',
        '_wheel_drive_velocity_cmd',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'wheel_drive_position': 'sequence<double>',
        'wheel_drive_velocity': 'sequence<double>',
        'wheel_steer_position': 'sequence<double>',
        'wheel_icr': 'sequence<double>',
        'wheel_steering_angle_cmd': 'sequence<double>',
        'wheel_drive_velocity_cmd': 'sequence<double>',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('double')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('double')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('double')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('double')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('double')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('double')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        if 'check_fields' in kwargs:
            self._check_fields = kwargs['check_fields']
        else:
            self._check_fields = ros_python_check_fields == '1'
        if self._check_fields:
            assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
                'Invalid arguments passed to constructor: %s' % \
                ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.wheel_drive_position = array.array('d', kwargs.get('wheel_drive_position', []))
        self.wheel_drive_velocity = array.array('d', kwargs.get('wheel_drive_velocity', []))
        self.wheel_steer_position = array.array('d', kwargs.get('wheel_steer_position', []))
        self.wheel_icr = array.array('d', kwargs.get('wheel_icr', []))
        self.wheel_steering_angle_cmd = array.array('d', kwargs.get('wheel_steering_angle_cmd', []))
        self.wheel_drive_velocity_cmd = array.array('d', kwargs.get('wheel_drive_velocity_cmd', []))

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.get_fields_and_field_types().keys(), self.SLOT_TYPES):
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
                    if self._check_fields:
                        assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.header != other.header:
            return False
        if self.wheel_drive_position != other.wheel_drive_position:
            return False
        if self.wheel_drive_velocity != other.wheel_drive_velocity:
            return False
        if self.wheel_steer_position != other.wheel_steer_position:
            return False
        if self.wheel_icr != other.wheel_icr:
            return False
        if self.wheel_steering_angle_cmd != other.wheel_steering_angle_cmd:
            return False
        if self.wheel_drive_velocity_cmd != other.wheel_drive_velocity_cmd:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if self._check_fields:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @builtins.property
    def wheel_drive_position(self):
        """Message field 'wheel_drive_position'."""
        return self._wheel_drive_position

    @wheel_drive_position.setter
    def wheel_drive_position(self, value):
        if self._check_fields:
            if isinstance(value, array.array):
                assert value.typecode == 'd', \
                    "The 'wheel_drive_position' array.array() must have the type code of 'd'"
                self._wheel_drive_position = value
                return
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'wheel_drive_position' field must be a set or sequence and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._wheel_drive_position = array.array('d', value)

    @builtins.property
    def wheel_drive_velocity(self):
        """Message field 'wheel_drive_velocity'."""
        return self._wheel_drive_velocity

    @wheel_drive_velocity.setter
    def wheel_drive_velocity(self, value):
        if self._check_fields:
            if isinstance(value, array.array):
                assert value.typecode == 'd', \
                    "The 'wheel_drive_velocity' array.array() must have the type code of 'd'"
                self._wheel_drive_velocity = value
                return
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'wheel_drive_velocity' field must be a set or sequence and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._wheel_drive_velocity = array.array('d', value)

    @builtins.property
    def wheel_steer_position(self):
        """Message field 'wheel_steer_position'."""
        return self._wheel_steer_position

    @wheel_steer_position.setter
    def wheel_steer_position(self, value):
        if self._check_fields:
            if isinstance(value, array.array):
                assert value.typecode == 'd', \
                    "The 'wheel_steer_position' array.array() must have the type code of 'd'"
                self._wheel_steer_position = value
                return
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'wheel_steer_position' field must be a set or sequence and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._wheel_steer_position = array.array('d', value)

    @builtins.property
    def wheel_icr(self):
        """Message field 'wheel_icr'."""
        return self._wheel_icr

    @wheel_icr.setter
    def wheel_icr(self, value):
        if self._check_fields:
            if isinstance(value, array.array):
                assert value.typecode == 'd', \
                    "The 'wheel_icr' array.array() must have the type code of 'd'"
                self._wheel_icr = value
                return
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'wheel_icr' field must be a set or sequence and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._wheel_icr = array.array('d', value)

    @builtins.property
    def wheel_steering_angle_cmd(self):
        """Message field 'wheel_steering_angle_cmd'."""
        return self._wheel_steering_angle_cmd

    @wheel_steering_angle_cmd.setter
    def wheel_steering_angle_cmd(self, value):
        if self._check_fields:
            if isinstance(value, array.array):
                assert value.typecode == 'd', \
                    "The 'wheel_steering_angle_cmd' array.array() must have the type code of 'd'"
                self._wheel_steering_angle_cmd = value
                return
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'wheel_steering_angle_cmd' field must be a set or sequence and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._wheel_steering_angle_cmd = array.array('d', value)

    @builtins.property
    def wheel_drive_velocity_cmd(self):
        """Message field 'wheel_drive_velocity_cmd'."""
        return self._wheel_drive_velocity_cmd

    @wheel_drive_velocity_cmd.setter
    def wheel_drive_velocity_cmd(self, value):
        if self._check_fields:
            if isinstance(value, array.array):
                assert value.typecode == 'd', \
                    "The 'wheel_drive_velocity_cmd' array.array() must have the type code of 'd'"
                self._wheel_drive_velocity_cmd = value
                return
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'wheel_drive_velocity_cmd' field must be a set or sequence and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._wheel_drive_velocity_cmd = array.array('d', value)
