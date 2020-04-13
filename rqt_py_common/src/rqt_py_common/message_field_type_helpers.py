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

SEQUENCE_PREFIX = 'sequence<'
SEQUENCE_PREFIX_LEN = len(SEQUENCE_PREFIX)

__LOGGER = logging.get_logger('message_field_type_helpers')

# leters, integers, and underscore  a slash then more leters, integers, and underscore
# valid_package_name_re = VALID_PACKAGE_NAME_RE.pattern
# if valid_package_name_re == '^[a-z]([a-z0-9_]?[a-z0-9]+)*$'


# Get valid generated msg regex from parser.py
VALID_PACKAGE_NAME_PATTERN = '[a-z]([a-z0-9_]?[a-z0-9]+)*'
VALID_MESSAGE_NAME_PATTERN = '[A-Z][A-Za-z0-9]*'
GENERATED_MSG_RE = VALID_PACKAGE_NAME_PATTERN + '/' + VALID_MESSAGE_NAME_PATTERN

BASIC_FIELD_TYPES = [
        'boolean', 'octet', 'float', 'double', 'uint8', 'int8', 'int16',
        'int32', 'int64', 'uint8', 'uint16', 'uint32', 'uint64',
        'string', 'wstring']

BASIC_FIELD_TYPE_RE = '(' + ')|('.join(BASIC_FIELD_TYPES) + ')'

# String followed by a number in <>
BOUNDED_STRING_RE = 'w?string<[1-9]+[0-9]*>'

# Any of the above three
VALID_BASE_FIELD_TYPE_RE = \
    BASIC_FIELD_TYPE_RE + '|(' + \
    GENERATED_MSG_RE + ')|(' + \
    BOUNDED_STRING_RE + ')'

# A valid msg followed by some number in brackets
STATIC_ARRAY_RE = '(' + VALID_BASE_FIELD_TYPE_RE + ")\[[1-9]+[0-9]*\]"

# A valid msg with a prefix of 'sequence<' and a suffix of '>'
UNBOUNDED_ARRAY_RE = 'sequence<(' + VALID_BASE_FIELD_TYPE_RE + ')>'

# A valid msg with a prefix of 'sequence<' and a suffix of ', [0-9]+>'
BOUNDED_ARRAY_RE = 'sequence<' + "(" + VALID_BASE_FIELD_TYPE_RE + ")" + ', [1-9]+[0-9]*>'

# Any of VALID_BASE_FIELD_TYPE_RE, STATIC_ARRAY_RE, UNBOUNDED_ARRAY_RE, or BOUNDED_ARRAY_RE
VALID_FIELD_TYPE_RE = \
    '(' + VALID_BASE_FIELD_TYPE_RE + ')|(' + STATIC_ARRAY_RE + ')|(' + \
    UNBOUNDED_ARRAY_RE + ')|(' + BOUNDED_ARRAY_RE + ')'

COMPILED_STATIC_ARRAY_RE = re.compile(STATIC_ARRAY_RE)
COMPILED_UNBOUNDED_ARRAY_RE = re.compile(UNBOUNDED_ARRAY_RE)
COMPILED_BOUNDED_ARRAY_RE = re.compile(BOUNDED_ARRAY_RE)
COMPILED_VALID_FIELD_TYPE_RE = re.compile(VALID_FIELD_TYPE_RE)
COMPILED_BOUNDED_STRING_RE = re.compile(BOUNDED_STRING_RE)

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

    SLOTS = (
        'is_valid',
        'base_type_str',
        'is_array',
        'is_static_array',
        'static_array_size',
        'is_bounded_array',
        'bounded_array_size',
        'is_unbounded_array',
        'is_bounded_string',
        'bounded_string_size',
    )

    def __init__(self, field_type: str):
        """
        Create a MessageFieldTypeInfo object from a field_type string

        :param field_type: the field type in the form generated by
            msg.get_fields_and_field_types
        """
        super(MessageFieldTypeInfo, self).__init__()

        self.__field_type = field_type

    @property
    def field_type(self):
        """Get the field type string for this MessageFieldTypeInfo"""
        return self.__field_type

    @field_type.setter
    def field_type(self, field_type: str) -> str:
        self = MessageFieldTypeInfo(field_type)

    @property
    def is_valid(self):
        """See docstring for message_field_type_helpers.is_valid"""
        if not hasattr(self, "_is_valid"):
            self._is_valid = is_valid(self.__field_type)
            # If this is not valid, then we set all the other member values
            if not self._is_valid:
                # We initialize these but do not populate them until they are accessed
                self._base_type_str = ""
                self._is_array = False
                self._is_static_array = False
                self._static_array_size = None
                self._is_bounded_array = False
                self._bounded_array_size = None
                self._is_unbounded_array = False
                self._is_bounded_string = False
                self._bounded_string_size = None

        return getattr(self, "_is_valid")

    @property
    def base_type_str(self):
        """See docstring for message_field_type_helpers.base_type_str"""
        if self.is_valid and not hasattr(self, "_base_type_str"):
            self._base_type_str = base_type_str(self.__field_type)
        return getattr(self, "_base_type_str")

    @property
    def is_array(self):
        """See docstring for message_field_type_helpers.is_array"""
        if self.is_valid and not hasattr(self, "_is_array"):
            # Note: we don't want to lazy compute this so we use logical inversion
            self._is_array = (
                self.is_static_array or \
                self.is_bounded_array or \
                self.is_unbounded_array
            )
        return getattr(self, "_is_array")

    @property
    def is_static_array(self):
        """See docstring for message_field_type_helpers.is_static_array"""
        if self.is_valid and not hasattr(self, "_is_static_array"):
            self._is_static_array = is_static_array(self.__field_type)
        return getattr(self, "_is_static_array")

    @property
    def static_array_size(self):
        """See docstring for message_field_type_helpers.static_array_size"""
        if self.is_valid and self.is_static_array:
            if not hasattr(self, "_static_array_size"):
                self._static_array_size = static_array_size(self.__field_type)
        else:
            self._static_array_size = None

        return getattr(self, "_static_array_size")

    @property
    def is_bounded_array(self):
        """See docstring for message_field_type_helpers.is_bounded_array"""
        if self.is_valid and not hasattr(self, "_is_bounded_array"):
            self._is_bounded_array = is_bounded_array(self.__field_type)
        return getattr(self, "_is_bounded_array")

    @property
    def bounded_array_size(self):
        """See docstring for message_field_type_helpers.bounded_array_size"""
        if self.is_valid and self.is_bounded_array:
            if not hasattr(self, "_bounded_array_size"):
                self._bounded_array_size = bounded_array_size(self.__field_type)
        else:
            self._bounded_array_size = None

        return getattr(self, "_bounded_array_size")

    @property
    def is_unbounded_array(self):
        """See docstring for message_field_type_helpers.is_unbounded_array"""
        if self.is_valid and not hasattr(self, "_is_unbounded_array"):
            self._is_unbounded_array = is_unbounded_array(self.__field_type)
        return getattr(self, "_is_unbounded_array")

    @property
    def is_bounded_string(self):
        """See docstring for message_field_type_helpers.is_bounded_string"""
        if self.is_valid and not hasattr(self, "_is_bounded_string"):
            self._is_bounded_string = is_bounded_string(self.__field_type)
        return getattr(self, "_is_bounded_string")

    @property
    def bounded_string_size(self):
        """See docstring for message_field_type_helpers.bounded_string_size"""
        if self.is_valid and self.is_bounded_string:
            if not hasattr(self, "_bounded_string_size"):
                self._bounded_string_size = bounded_string_size(self.__field_type)
        else:
            self._bounded_string_size = None

        return getattr(self, "_bounded_string_size")

    def to_dict(self) -> Dict[str, Any]:
        """Returns the type info as a dictionary"""
        info_as_dict = {}
        # We must call self.is_valid first to populate the dict for an invalid
        # self.__type_name
        is_valid = self.is_valid
        for slot in MessageFieldTypeInfo.SLOTS:
            info_as_dict[slot] = getattr(self, slot)

        return info_as_dict


def is_valid(field_type: str) -> bool:
    """Check if the field type is valid"""
    return COMPILED_VALID_FIELD_TYPE_RE.fullmatch(field_type) is not None

def base_type_str(field_type: str) -> str:
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
    if not is_valid(field_type):
        return ''

    # Strip all array info from field type
    field_type = strip_array_from_field_type(field_type)
    # Strip the string boundedness info from field type
    string_bound_delim_ix = field_type.find('<')
    if string_bound_delim_ix >= 0:
        field_type = field_type[:string_bound_delim_ix]

    return field_type

def is_array(field_type: str) -> bool:
    """
    returns true if the field_type is a static, bounded or unbounded array

    :param field_type: Field type as produced by get_fields_and_field_types()

    :returns: If the field type is one of static, bounded or unbounded array
    """
    return \
        is_static_array(field_type) or \
        is_bounded_array(field_type) or \
        is_unbounded_array(field_type)

def is_static_array(field_type: str) -> bool:
    """
    Check if field_Type is a static array

    eg:
        uint8[4] -> True
        sequence<uint8, 5> -> False

    :param field_type: Field type as produced by get_fields_and_field_types()

    :returns: True if the field_type is a static sized array
    """
    return COMPILED_STATIC_ARRAY_RE.fullmatch(field_type) is not None

def static_array_size(field_type, check_valid: bool = True) -> Optional[int]:
    """
    If the field_type is a static array get the static array size

    ie: If `is_static_array == True` then `len(msg.field_type)`
        If not a static array, return None
    """
    if check_valid and not is_static_array(field_type):
        return None

    start_ix = field_type.rfind('[')
    # Should always be the last index
    end_ix = field_type.rfind(']')
    try:
        return int(field_type[start_ix + 1:end_ix])

    except ValueError:
        pass

    return None


def is_bounded_array(field_type: str) -> bool:
    """Is the field_type a bounded array"""
    return COMPILED_BOUNDED_ARRAY_RE.fullmatch(field_type) is not None

def bounded_array_size(field_type: str, check_valid: bool = True) -> Optional[int]:
    """
    If the field_type is sequence<some_val, some_val> return some_val as int

    :returns: The bounded_array_size as int, or None if not a bounded_array
    """
    if check_valid and not is_bounded_array(field_type):
        return None

    bounded_size_start_ix = field_type.find(', ')
    try:
        return int(field_type[bounded_size_start_ix + 2:-1])
    except ValueError:
        return None

def is_unbounded_array(field_type: str) -> bool:
    """Check if the field_type is an unbounded array"""
    return COMPILED_UNBOUNDED_ARRAY_RE.fullmatch(field_type) is not None

def is_bounded_string(field_type: str) -> bool:
    """Check if the field_type has string<.* return true"""
    return \
        is_valid(field_type) and \
        COMPILED_BOUNDED_STRING_RE.search(field_type) is not None

def strip_array_from_field_type(field_type: str) -> str:
    """
    Remove static, bounded and unbounded array info from field_type

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

    if not is_valid(field_type):
        return ''

    if field_type.startswith(SEQUENCE_PREFIX):
        # Check for bound sequence
        seq_end_ix = field_type.rfind(',')
        # If not bound check for unbound sequence
        if seq_end_ix < 0:
            seq_end_ix = field_type.rfind('>')

        # If we have found an end to the sequence, then we strip that
        if seq_end_ix > SEQUENCE_PREFIX_LEN:
            field_type = field_type[SEQUENCE_PREFIX_LEN:seq_end_ix]

    # Check for static arrays and strip those out
    static_array_start_ix = field_type.rfind('[')
    if static_array_start_ix >= 0:
        field_type = field_type[:static_array_start_ix]

    return field_type

__BOUNDED_STRING_DELIM = 'string<'
def bounded_string_size(field_type: str, check_valid: bool = True) -> Optional[int]:
    """Maximum length of a string in the field, (-1 if not string)"""
    if check_valid and not is_bounded_string(field_type):
        return None

    field_type = strip_array_from_field_type(field_type)
    start_ix = field_type.find(__BOUNDED_STRING_DELIM)
    if start_ix >= 0:
        start_ix += len(__BOUNDED_STRING_DELIM)
        end_ix = field_type.rfind('>')
        try:
            return int(field_type[start_ix:end_ix])
        except ValueError:
                pass
    return None


def is_primitive_type(field_type: str) -> bool:
    """Checks if the field type is in PRIMITIVE_TYPES"""
    return field_type in PRIMITIVE_TYPES

def is_base_type_primitive_type(field_type: str) -> bool:
    """Checks if the base field type is a primitive type"""
    # First remove any unused information about sequences etc..
    field_type = base_type_str(field_type)
    return is_primitive_type(field_type)

def get_base_python_type(field_type: str) -> Any:
    """
    Get the python class of the base field type
    eg:
        `sequence<my_msgs/Custom, 5>`   --> my_msgs.msg.Custom
        `sequence<uint8, 5>`            --> int
        `sequence<string<4>, 5>`        --> str

    :returns: The class of the base python type or None if field_type is invalid
    """
    field_type = base_type_str(field_type)
    class_type = get_type_class(field_type)
    if class_type is None:
        class_type = get_message_class(field_type)

    return class_type

def get_type_class(field_type: str):
    """
    Gets the python type from an idl string.

    See: https://github.com/ros2/design/blob/gh-pages/articles/142_idl.md

    @param field_type: the IDL type of field
    @type message_type: str
    """

    if field_type in FLOATING_POINT_TYPES:
        return float

    elif field_type in STRING_TYPES:
        return str

    elif field_type in OCTET_TYPE:
        return bytes

    elif field_type in INTEGER_TYPES:
        return int

    elif field_type in BOOLEAN_TYPE:
        return bool

    else:
        return None

def separate_field_from_array_information(field_name: str) -> Tuple[str, bool, Optional[int]]:
    """
    Separates the mesage slot name from the index information

    eg:
        /positions[0]       -> /positions, True, 0
        /positions[0]/pos   -> /positions[0]/pos, False, None
        /positions          -> /positions, False, None

    If the input is malformed, (eg. `/positions[[0]`, we return '', False, None )
    :returns: the slot name, is_array, index
    """
    is_indexed = False
    index = None

    # Check for array index information
    if field_name[-1] != ']':
        return field_name, is_indexed, index

    field_name_tokens = field_name.rsplit('[', 1)
    if len(field_name_tokens) > 1:
        is_indexed = True
        try:
            index = int(field_name_tokens[1].rstrip(']'))
        except ValueError:
            return "", False, None

    return field_name_tokens[0], is_indexed, index

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
    logger = __LOGGER.get_child("get_slot_class_and_field_information")

    if is_primitive_type(message_class):
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
            message_class = get_base_python_type(slot_class_str)
            if not message_class:
                logger.warn("could not find python class for: [%s]" % slot_class_str)
                return None, None

    if is_indexed:
        slot_class_str = strip_array_from_field_type(slot_class_str)

    if (slot_class_str):
        array_info = \
            MessageFieldTypeInfo(slot_class_str)

    return message_class, array_info
