# Copyright (c) 2020, Michael Lautman PickNik Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the TU Darmstadt nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Author: Michael Lautman
# This functionality was ported from ROS1. For the original soure code:
#   https://github.com/ros/ros_comm/blob/noetic-devel/tools/rostopic/src/rostopic/__init__.py

import re

from typing import Dict, TypeVar, Tuple, Any, Optional

from rclpy import logging

from rqt_py_common.message_helpers import get_message_class
from rqt_py_common.topic_helpers import separate_field_from_array_information
from python_qt_binding.QtCore import qWarning

from rosidl_parser.definition import BASIC_TYPES, INTEGER_TYPES, \
                                     FLOATING_POINT_TYPES, CHARACTER_TYPES, \
                                     BOOLEAN_TYPE, OCTET_TYPE

MsgType = TypeVar('MsgType')

# Make sure all types are tuples
if type(BASIC_TYPES) is str:
    BASIC_TYPES = (BASIC_TYPES,)

if type(INTEGER_TYPES) is str:
    INTEGER_TYPES = (INTEGER_TYPES,)

if type(FLOATING_POINT_TYPES) is str:
    FLOATING_POINT_TYPES = (FLOATING_POINT_TYPES,)

if type(CHARACTER_TYPES) is str:
    CHARACTER_TYPES = (CHARACTER_TYPES,)

if type(BOOLEAN_TYPE) is str:
    BOOLEAN_TYPE = (BOOLEAN_TYPE,)

if type(OCTET_TYPE) is str:
    OCTET_TYPE = (OCTET_TYPE,)

if type(BOOLEAN_TYPE) is str:
    BOOLEAN_TYPE = (BOOLEAN_TYPE,)

# Create our own Types
BASIC_STRING_TYPES = (
    'string',
    'wstring'
)

PRIMITIVE_TYPES = (
    *BASIC_TYPES,
    *BASIC_STRING_TYPES
)

STRING_TYPES = (
    *BASIC_STRING_TYPES,
    *CHARACTER_TYPES
)

_LOGGER = logging.get_logger('message_field_type_helpers')


class MessageFieldTypeInfo(object):
    """
    Useful helper methods for working with a message's field types

    Parse items such as: (from rosidl_generator_py definitions)
      .msg            field_type                    python
      -----------------------------------------------------------
      bool            -> 'boolean'                  -> bool
      byte            -> 'octet'                    -> bytes
      float32         -> 'float'                    -> float
      float64         -> 'double'                   -> float
      char            -> 'uint8'                    -> int
      int8            -> 'int8'                     -> int
      int16           -> 'int16'                    -> int
      int32           -> 'int32'                    -> int
      int64           -> 'int64'                    -> int
      uint8           -> 'uint8'                    -> int
      uint16          -> 'uint16'                   -> int
      uint32          -> 'uint32'                   -> int
      uint64          -> 'uint64'                   -> int
      string          -> 'string'                   -> str
      string          -> 'wstring'                  -> str
      my_msgs/Custom  -> 'my_msgs/Custom'           -> my_msgs.msg.Custom

      # Arrays with normal values
      int8[3]         -> 'int8[3]'                  -> List[int]
      int8[]          -> 'sequence<int8>'           -> List[int]
      int8[<=3]       -> 'sequence<int8, 3>'        -> List[int]

      # String weirdness
      string<=5       -> 'string<5>'                -> str
      string[3]       -> 'string[3]'                -> List[str]
      string[]        -> 'sequence<string>'         -> List[str]
      string<=5[3]    -> 'string<5>[3]'             -> List[str]
      string<=5[<=10] -> 'sequence<string<5>, 10>'  -> List[str]
      string<=5[]     -> 'sequence<string<5>>'      -> List[str]
      string[<=10]    -> 'sequence<string, 10>'     -> List[str]

      # WString weirdness
      wstring<=5       -> 'wstring<5>'                -> str
      wstring[3]       -> 'wstring[3]'                -> List[str]
      wstring[]        -> 'sequence<wstring>'         -> List[str]
      wstring<=5[3]    -> 'wstring<5>[3]'             -> List[str]
      wstring<=5[<=10] -> 'sequence<wstring<5>, 10>'  -> List[str]
      wstring<=5[]     -> 'sequence<wstring<5>>'      -> List[str]
      wstring[<=10]    -> 'sequence<wstring, 10>'     -> List[str]
    """

    # In addition to pre-allocating storage for these values, this is used by
    # get_field_type_attributes getting access to the list of available
    # attributes which are lazy initialized
    __slots__ = (
        '_field_type',
        '_is_valid',
        '_base_type_str',
        '_is_array',
        '_is_static_array',
        '_static_array_size',
        '_is_bounded_array',
        '_bounded_array_size',
        '_is_unbounded_array',
        '_is_bounded_string',
        '_bounded_string_size',
        '_field_type_without_array_info'
    )

    __logger = _LOGGER.get_child("MessageFieldTypeInfo")

    # Deliminators for bounded string
    __BOUNDED_STRING_DELIM = 'string<'
    __BOUNDED_STRING_DELIM_LEN = len(__BOUNDED_STRING_DELIM)

    # prefix for bound and unbound arrays
    _SEQUENCE_PREFIX = 'sequence<'
    _SEQUENCE_PREFIX_LEN = len(_SEQUENCE_PREFIX)

    # Valid generated msg pattern
    _VALID_PACKAGE_NAME_RE = '[a-z]([a-z0-9_]?[a-z0-9]+)*'
    _VALID_MESSAGE_NAME_RE = '[A-Z][A-Za-z0-9]*'
    _GENERATED_MSG_RE = _VALID_PACKAGE_NAME_RE + '/' + _VALID_MESSAGE_NAME_RE

    _BASIC_FIELD_TYPES = [
            'boolean', 'octet', 'float', 'double', 'uint8', 'int8', 'int16',
            'int32', 'int64', 'uint8', 'uint16', 'uint32', 'uint64',
            'string', 'wstring']

    _BASIC_FIELD_TYPES_RE = '(' + ')|('.join(_BASIC_FIELD_TYPES) + ')'

    # String followed by a number in <>
    _BOUNDED_STRING_RE = 'w?string<[1-9]+[0-9]*>'

    # Any of the above three
    _VALID_BASE_FIELD_TYPE_RE = \
        _BASIC_FIELD_TYPES_RE + '|(' + \
        _GENERATED_MSG_RE + ')|(' + \
        _BOUNDED_STRING_RE + ')'

    # A valid msg followed by some number in brackets
    _STATIC_ARRAY_RE = '(' + _VALID_BASE_FIELD_TYPE_RE + ")\[[1-9]+[0-9]*\]"

    # A valid msg with a prefix of 'sequence<' and a suffix of '>'
    _UNBOUNDED_ARRAY_RE = 'sequence<(' + _VALID_BASE_FIELD_TYPE_RE + ')>'

    # A valid msg with a prefix of 'sequence<' and a suffix of ', [0-9]+>'
    _BOUNDED_ARRAY_RE = 'sequence<' + "(" + _VALID_BASE_FIELD_TYPE_RE + ")" + ', [1-9]+[0-9]*>'

    # Any of _VALID_BASE_FIELD_TYPE_RE, _STATIC_ARRAY_RE, _UNBOUNDED_ARRAY_RE, or _BOUNDED_ARRAY_RE
    _VALID_FIELD_TYPE_RE = \
        '(' + _VALID_BASE_FIELD_TYPE_RE + ')|(' + _STATIC_ARRAY_RE + ')|(' + \
        _UNBOUNDED_ARRAY_RE + ')|(' + _BOUNDED_ARRAY_RE + ')'

    _COMPILED_STATIC_ARRAY_RE = re.compile(_STATIC_ARRAY_RE)
    _COMPILED_UNBOUNDED_ARRAY_RE = re.compile(_UNBOUNDED_ARRAY_RE)
    _COMPILED_BOUNDED_ARRAY_RE = re.compile(_BOUNDED_ARRAY_RE)
    _COMPILED_VALID_FIELD_TYPE_RE = re.compile(_VALID_FIELD_TYPE_RE)
    _COMPILED_BOUNDED_STRING_RE = re.compile(_BOUNDED_STRING_RE)

    def __init__(self, field_type: str):
        """Create a MessageFieldTypeInfo object from a field_type string"""
        super(MessageFieldTypeInfo, self).__init__()

        self._field_type = field_type

    @staticmethod
    def get_field_type_attributes():
        """Get a tuple of attributes names for this class"""
        attributes = tuple(
            att.lstrip('_') for att in MessageFieldTypeInfo.__slots__
        )
        return attributes


    def to_dict(self) -> Dict[str, Any]:
        """Returns the field type info info as a dictionary"""
        info_as_dict = {}
        for slot in MessageFieldTypeInfo.get_field_type_attributes():
            info_as_dict[slot] = getattr(self, slot)

        return info_as_dict

    @property
    def field_type(self):
        """Get the field type string for this MessageFieldTypeInfo"""
        return self._field_type

    @field_type.setter
    def field_type(self, field_type: str) -> str:
        """Set the field type and reset the field type info attributes"""
        self = MessageFieldTypeInfo(field_type)

    @property
    def is_valid(self):
        """Check if the field type is valid according to the idl spec"""
        # Lazy initialization
        if not hasattr(self, "_is_valid"):
            self._is_valid = \
                self._COMPILED_VALID_FIELD_TYPE_RE.fullmatch(self._field_type) \
                is not None

        return getattr(self, "_is_valid")

    @property
    def base_type_str(self):
        """
        Get the base type of field_type by removing the array and string length info

        eg:
          .msg definition : field_type                 -> output
          -----------------------------------------------------------
          int8[3]         : 'int8[3]'                  -> 'int'
          int8[]          : 'sequence<int8>'           -> 'int'
          int8[<=3]       : 'sequence<int8, 3>'        -> 'int'
          string          : 'string'                   -> 'string'
          wstring         : 'wstring'                  -> 'wstring'
          string[3]       : 'string[3]'                -> 'string'
          string[]        : 'sequence<string>'         -> 'string'
          string<=5       : 'string<5>'                -> 'string'
          string<=5[3]    : 'string<5>[3]'             -> 'string'
          string<=5[<=10] : 'sequence<string<5>, 10>'  -> 'string'
          string<=5[]     : 'sequence<string<5>>'      -> 'string'
          string[<=10]    : 'sequence<string, 10>'     -> 'string'

        :param field_type: The field_type such as one as created by
            get_fields_and_field_types()

        :returns: the field_type of with the list component stripped out.
            If field_type is not valid then, it returns an empty string
        """
        # Lazy initialization
        if not self.is_valid:
            self._base_type_str = ""

        elif not hasattr(self, "_base_type_str"):
            # Strip all array info from field type and set the attribute
            self._base_type_str = self.field_type_without_array_info
            # Strip the string boundedness info from field type
            string_bound_delim_ix = self._base_type_str.find('<')
            if string_bound_delim_ix >= 0:
                self._base_type_str = \
                    self._base_type_str[:string_bound_delim_ix]

        return getattr(self, "_base_type_str")

    @property
    def is_array(self):
        """
        Check if the field_type is a static, bounded or unbounded array

        :returns: If the field type is one of static, bounded or unbounded array
        """
        # Lazy initialization
        if not self.is_valid:
            self._is_array = False

        elif not hasattr(self, "_is_array"):
            self._is_array = (
                self.is_static_array or \
                self.is_bounded_array or \
                self.is_unbounded_array
            )

        return getattr(self, "_is_array")

    @property
    def is_static_array(self):
        """
        Check if field_Type is a static array

        eg:
            uint8[4] -> True
            sequence<uint8, 5> -> False

        :returns: True if the field_type is a static sized array
        """
        # Lazy initialization
        if not self.is_valid:
            self._is_static_array = False

        elif not hasattr(self, "_is_static_array"):
            self._is_static_array  = \
                self._COMPILED_STATIC_ARRAY_RE.fullmatch(self._field_type) \
                is not None

        return getattr(self, "_is_static_array")

    @property
    def static_array_size(self):
        """
        If the field_type is a static array get the static array size

        ie: If `is_static_array == True` then `len(msg.field_type)`
            If not a static array, return None
        """
        # Lazy initialization
        if not self.is_valid or not self.is_static_array:
            self._static_array_size = None

        elif not hasattr(self, "_static_array_size"):
            start_ix = self._field_type.rfind('[') + 1
            # Should always be the last index
            end_ix = self._field_type.rfind(']')
            try:
                self._static_array_size = \
                    int(self._field_type[start_ix:end_ix])

            except ValueError:
                self.__logger.get_child('static_array_size').warn(
                    'ValueError when parsing [%s]' % self._field_type
                )
                self._static_array_size = None

        return getattr(self, "_static_array_size")

    @property
    def is_bounded_array(self):
        """Is the field_type a bounded array"""
        # Lazy initialization
        if not self.is_valid:
            self._is_bounded_array = False

        elif not hasattr(self, "_is_bounded_array"):
            self._is_bounded_array = \
                self._COMPILED_BOUNDED_ARRAY_RE.fullmatch(self._field_type) \
                is not None

        return getattr(self, "_is_bounded_array")

    @property
    def bounded_array_size(self):
        """
        If the field_type is sequence<some_val, some_val> return some_val as int

        :returns: The bounded_array_size as int, or None if not a bounded_array
        """
        # Lazy initialization
        if not self.is_valid or not self.is_bounded_array:
            self._bounded_array_size = None

        elif not hasattr(self, "_bounded_array_size"):
            bounded_size_start_ix = self._field_type.rfind(', ') + 2
            try:
                self._bounded_array_size = \
                    int(self._field_type[bounded_size_start_ix:-1])
            except ValueError:
                self.__logger.get_child('static_array_size').warn(
                    'ValueError when parsing [%s]' % self._field_type
                )
                self._bounded_array_size = None

        return getattr(self, "_bounded_array_size")

    @property
    def is_unbounded_array(self):
        """Check if the field_type is an unbounded array"""
        # Lazy initialization
        if not self.is_valid:
            self._is_unbounded_array = False

        elif not hasattr(self, "_is_unbounded_array"):
            self._is_unbounded_array = \
                self._COMPILED_UNBOUNDED_ARRAY_RE.fullmatch(self._field_type) \
                is not None

        return getattr(self, "_is_unbounded_array")

    @property
    def is_bounded_string(self):
        """Check if the field_type has string<.* return true"""
        # Lazy initialization
        if not self.is_valid:
            self._is_bounded_string = False

        elif not hasattr(self, "_is_bounded_string"):
            self._is_bounded_string = \
                self._COMPILED_BOUNDED_STRING_RE.search(self._field_type) \
                is not None

        return getattr(self, "_is_bounded_string")

    @property
    def bounded_string_size(self):
        """Maximum length of a string in the field, (-1 if not string)"""
        # Lazy initialization
        if not self.is_valid or not self.is_bounded_string:
            self._bounded_string_size = None

        elif not hasattr(self, "_bounded_string_size"):
            match = self._COMPILED_BOUNDED_STRING_RE.search(self._field_type)
            # this should never be none
            if match is None:
                self.__logger.get_child('bounded_string_size').warn(
                    'Found no matches for _COMPILED_BOUNDED_STRING_RE in [%s]' %
                     self._field_type
                )
                self._bounded_string_size = None
            else:
                match_s = match.group()
                size_start = match_s.find('<') + 1
                size_end = match_s.find('>')
                try:
                    self._bounded_string_size  = \
                        int(match_s[size_start:size_end])
                except ValueError:
                    self.__logger.get_child('bounded_string_size').warn(
                        'ValueError when parsing [%s]' % self._field_type
                    )
                    self._bounded_string_size = None

        return getattr(self, "_bounded_string_size")

    @property
    def field_type_without_array_info(self) -> str:
        """
        Get the field type with static, bounded and unbounded array info removed

        eg:
          .msg definition : field_type                 -> output
          -----------------------------------------------------------
          int8[3]         : 'int8[3]'                ,  -> 'int'
          int8[]          : 'sequence<int8>'         ,  -> 'int'
          int8[<=3]       : 'sequence<int8, 3>'      ,  -> 'int'
          string          : 'string'                 ,  -> 'string'
          string[3]       : 'string[3]'              ,  -> 'string'
          string[]        : 'sequence<string>'       ,  -> 'string'
          string<=5       : 'string<5>'              ,  -> 'string<5>'
          string<=5[3]    : 'string<5>[3]'           ,  -> 'string<5>'
          string<=5[<=10] : 'sequence<string<5>, 10>',  -> 'string<5>'
          string<=5[]     : 'sequence<string<5>>'    ,  -> 'string<5>'
          string[<=10]    : 'sequence<string, 10>'   ,  -> 'string'

        :returns: The base field type preserving string boundedness if appropriate.
            If not a valid field_type, then return the empty string
        """
        if not self.is_valid:
            self._field_type_without_array_info = ''

        elif not hasattr(self, '_field_type_without_array_info'):
            # Not an array
            if not self.is_array:
                self._field_type_without_array_info = self._field_type

            # Bound / Unbound sequence
            elif self._field_type.startswith(self._SEQUENCE_PREFIX):
                # First check for bound sequence since that will be shorter
                seq_end_ix = self._field_type.rfind(',')

                # If not bounded check for unbound sequence
                if seq_end_ix < 0:
                    seq_end_ix = self._field_type.rfind('>')

                # If we have found an end to the sequence, then we strip that
                if seq_end_ix > self._SEQUENCE_PREFIX_LEN:
                    self._field_type_without_array_info = \
                        self._field_type[self._SEQUENCE_PREFIX_LEN:seq_end_ix]

            # Check for static arrays and strip those out
            else:
                static_array_start_ix = self._field_type.rfind('[')
                if static_array_start_ix >= 0:
                    self._field_type_without_array_info = \
                        self._field_type[:static_array_start_ix]
                else:
                    self.__logger.get_child(
                        'field_type_without_array_info'
                    ).warn(
                        'is_array is true but failed to parse: [%s]' % (
                            self._field_type
                        )
                    )
                    self._field_type_without_array_info = ''


        return getattr(self, '_field_type_without_array_info')


def is_primitive_type(field_type: str) -> bool:
    """Checks if the field type is in PRIMITIVE_TYPES"""
    return get_primitive_python_class(field_type) is not None


def class_is_primitive_type(field_class: Any) -> bool:
    """Checks if the field_class is a generated msg or not"""
    return not hasattr(field_class, 'get_fields_and_field_types')


_primitive_python_class_cache = {}
def get_primitive_python_class(field_type: str) -> Any:
    """
    Gets the python type from an idl string.

    See: https://github.com/ros2/design/blob/gh-pages/articles/142_idl.md

    @param field_type: the IDL type of field
    @type field_type: str
    """

    # Construct this cache the first time this method is called
    if not _primitive_python_class_cache:
        for t in FLOATING_POINT_TYPES:
            _primitive_python_class_cache[t] = float
        for t in STRING_TYPES:
            _primitive_python_class_cache[t] = str
        for t in OCTET_TYPE:
            _primitive_python_class_cache[t] = bytes
        for t in INTEGER_TYPES:
            _primitive_python_class_cache[t] = int
        for t in BOOLEAN_TYPE:
            _primitive_python_class_cache[t] = bool

    try:
        return _primitive_python_class_cache[field_type]
    except KeyError:
        mfti = MessageFieldTypeInfo(field_type)
        if not mfti.is_valid:
            return None

        if mfti.is_array:
            return list

        if mfti.is_bounded_string:
            return str

        return None


def get_field_python_class(field_type: str) -> Any:
    """
    Get the python class of the field type
    eg:
        `sequence<my_msgs/Custom, 5>`   --> my_msgs.msg.Custom
        `sequence<uint8, 5>`            --> int
        `sequence<string<4>, 5>`        --> str

    :returns: The class of the base python type or None if field_type is invalid
    """
    class_type = get_primitive_python_class(field_type)
    if class_type is None:
        class_type = get_message_class(field_type)

    return class_type


def get_field_base_python_class(field_type: str) -> Any:
    """
    Get the python class of the base field type
    eg:
        `sequence<my_msgs/Custom, 5>`   --> my_msgs.msg.Custom
        `sequence<uint8, 5>`            --> int
        `sequence<string<4>, 5>`        --> str

    :returns: The class of the base python type or None if field_type is invalid
    """
    return get_field_python_class(
        MessageFieldTypeInfo(field_type).base_type_str
    )


def get_slot_class_and_field_information(message_class: MsgType, slot_path: str) -> Tuple[Any, MessageFieldTypeInfo]:
    """
    Get the Python class of slot in the given message class and its field info

    If the field is an array, the type of the array's values are returned and the is_array flag is
    set to True. This is a static type check, so it works for unpublished topics and with empty
    arrays.

    we check if the message_class is primitive and if it is we return None, None

    :param message_class: message class type as a class.
    :param slot_path: path to the slot inside the message class, ``str``, i.e.
        'header/seq[1]' or '/header/seq/a'
        if empty string ('') then we return message_class and the field info fo message_class

    :returns: slot class, array_info where array_info is a map
    :rtype: class, map
    """
    logger = _LOGGER.get_child("get_slot_class_and_field_information")

    if class_is_primitive_type(message_class):
        logger.info("message_class: %s is primitive. Cannot get field information" % message_class)
        return None, None

    array_info = None
    slot_class_str = ""
    is_indexed = False

    fields = [f for f in slot_path.split('/') if f]
    if not len(fields):
        message_instance = message_class()
        # message_instance should be of form something like:
        #   'rqt_py_common.msg.Val(floats=array([ 0.,...
        # We isolate 'rqt_py_common.msg.Val' and replace '.msg.' with '/'
        slot_class_str = \
            "/".join(
                message_instance.__repr__().split("(", 1)[0].split(".msg.", 1)
            )
        logger.info("slot_path is empty, returning message_class info for: %s" % slot_class_str)

    else:
        # len(fields)> 0 indicates that we should traverse msg fields
        for field_name in fields:
            field_name, is_indexed, _ = \
                separate_field_from_array_information(field_name)

            message_class_slots = message_class.get_fields_and_field_types()
            if not field_name in message_class_slots:
                logger.warn(
                    "field: '%s' not in slots %s" % (field_name, message_class_slots))
                return None, None

            slot_class_str = message_class_slots[field_name]
            message_class = get_field_python_class(
                MessageFieldTypeInfo(slot_class_str).base_type_str
            )
            if not message_class:
                logger.warn("could not find python class for: [%s]" % slot_class_str)
                return None, None

    # Check if the last field in fields is an array and if so, return just the
    # type not the array
    if is_indexed:
        slot_class_str = \
            MessageFieldTypeInfo(slot_class_str).field_type_without_array_info

    if (slot_class_str):
        array_info = \
            MessageFieldTypeInfo(slot_class_str)

    return message_class, array_info
