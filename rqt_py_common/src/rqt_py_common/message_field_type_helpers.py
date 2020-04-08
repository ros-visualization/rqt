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

from typing import Dict, TypeVar, Tuple, Any

from rclpy import logging

from rqt_py_common.message_helpers import get_message_class
from python_qt_binding.QtCore import qWarning

from rosidl_parser.definition import BASIC_TYPES, INTEGER_TYPES, \
                                     FLOATING_POINT_TYPES, CHARACTER_TYPES, \
                                     BOOLEAN_TYPE, OCTET_TYPE

MsgType = TypeVar('MsgType')

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
GENERATED_MSG_RE = '[a-zA-Z0-9_]+/[a-zA-Z0-9_]+'
# letters followed by integers
BASIC_TYPE_RE = '[a-zA-Z]+[0-9]*'
# String followed by a number in <>
BOUNDED_STRING_RE = 'string<[0-9]+>'

# Any of the above three
VALID_BASE_MSG_RE = \
    '(' + GENERATED_MSG_RE + ')|(' + BASIC_TYPE_RE + \
    ')|(' + BOUNDED_STRING_RE + ')'

# A valid msg followed by some number in brackets
STATIC_ARRAY_RE = "(" + VALID_BASE_MSG_RE + ")" + "\[[0-9]+\]"

# A valid msg with a prefix of 'sequence<' and a suffix of '>'
UNBOUNDED_ARRAY_RE = 'sequence<' + "(" + VALID_BASE_MSG_RE + ")" + '>'

# A valid msg with a prefix of 'sequence<' and a suffix of ', [0-9]+>'
BOUNDED_ARRAY_RE = 'sequence<' + "(" + VALID_BASE_MSG_RE + ")" + ', [0-9]+>'

# Any of VALID_BASE_MSG_RE, STATIC_ARRAY_RE, UNBOUNDED_ARRAY_RE, or BOUNDED_ARRAY_RE
VALID_MSG_RE = \
    VALID_BASE_MSG_RE + '|(' + STATIC_ARRAY_RE + ')|(' + \
    UNBOUNDED_ARRAY_RE + ')|(' + BOUNDED_ARRAY_RE + ')'

COMPILED_STATIC_ARRAY_RE = re.compile(STATIC_ARRAY_RE)
COMPILED_UNBOUNDED_ARRAY_RE = re.compile(UNBOUNDED_ARRAY_RE)
COMPILED_BOUNDED_ARRAY_RE = re.compile(BOUNDED_ARRAY_RE)
COMPILED_VALID_MSG_RE = re.compile(VALID_MSG_RE)
COMPILED_BOUNDED_STRING_RE = re.compile(BOUNDED_STRING_RE)

class MessageFieldTypeInfo(object):

    __slots__ = (
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

    :param field_type: the field type in the form generated by msg.get_fields_and_field_types
    :type field_type: str
    """
    def __init__(self, field_type: str):
        super(MessageFieldTypeInfo, self).__init__()

        self.is_valid = False
        self.base_type_str = ""
        self.is_array = False
        self.is_static_array = False
        self.static_array_size = -1
        self.is_bounded_array = False
        self.bounded_array_size = -1
        self.is_unbounded_array = False
        self.is_bounded_string = False
        self.bounded_string_size = -1

        self.is_valid = is_valid(field_type)
        if self.is_valid:

            self.base_type_str = base_type_str(field_type)

            self.is_static_array = is_static_array(field_type)
            if self.is_static_array:
                self.static_array_size = static_array_size(field_type, check_valid=False)

            self.is_bounded_array = is_bounded_array(field_type)
            if self.is_bounded_array:
                self.bounded_array_size = bounded_array_size(field_type, check_valid=False)

            self.is_unbounded_array = is_unbounded_array(field_type)

            self.is_array = \
                self.is_static_array or \
                self.is_bounded_array or \
                self.is_unbounded_array

            self.is_bounded_string = is_bounded_string(field_type)
            if self.is_bounded_string:
                self.bounded_string_size = bounded_string_size(field_type, check_valid=False)

    def as_dict(self) -> Dict[str, Any]:
        """Returns the type info as a dictionary"""
        info_as_dict = {}
        for slot in MessageFieldTypeInfo.__slots__:
            info_as_dict[slot] = getattr(self, slot)
        return info_as_dict

def is_valid(field_type: str) -> bool:
    """Check if the field type is valid"""
    return COMPILED_VALID_MSG_RE.fullmatch(field_type) is not None

__END_OF_BASE_TYPE_DELIM = ['<', '[', ',', '>']
def base_type_str(field_type: str) -> str:
    """
    Get the base type of field_type by removing the array information

    eg:
        uint8[5] -> uint8
        sequence<my_msgs/Custom> -> my_msgs/Custom

    :param field_type: The field_type such as one as created by
        get_fields_and_field_types()

    :returns: the field_type of with the list component stripped out
    """
    if field_type.startswith(SEQUENCE_PREFIX):
        field_type = field_type[SEQUENCE_PREFIX_LEN:]

    for delim in __END_OF_BASE_TYPE_DELIM:
        delim_ix = field_type.find(delim)
        if delim_ix >= 0:
            field_type = field_type[:delim_ix]

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

def static_array_size(field_type, check_valid: bool = True) -> int:
    """
    If the field_type is a static array get the static array size

    ie: If `is_static_array == True` then `len(msg.field_type)`
        If not a static array, return -1
    """
    if check_valid and not is_static_array(field_type):
        return -1

    start_ix = field_type.find('[')
    end_ix = field_type.find(']')
    try:
        return int(field_type[start_ix + 1:end_ix])
    except ValueError:
        return -1

def is_bounded_array(field_type: str) -> bool:
    """
    Is the field_type a bounded array
    """
    return COMPILED_BOUNDED_ARRAY_RE.fullmatch(field_type) is not None

def bounded_array_size(field_type: str, check_valid: bool = True) -> int:
    """
    If the field_type is sequence<some_val, some_val> return some_val as int

    If not a bounded string, return -1
    """
    if check_valid and not is_bounded_array(field_type):
        return -1

    bounded_size_start_ix = field_type.find(', ')
    try:
        return int(field_type[bounded_size_start_ix + 2:-1])
    except ValueError:
        return -1

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
    Remove leading ''sequence<' and trailing '>' as well as sequence bounding limit

    eg:
      .msg definition : field_type                 -> output
      -----------------------------------------------------------
      int8[3]         : 'int8[3]'                  -> 'int'
      int8[]          : 'sequence<int8>'           -> 'int'
      int8[<=3]       : 'sequence<int8, 3>'        -> 'int'
      string          : 'string'                   -> 'string'
      string[3]       : 'string[3]'                -> 'string'
      string[]        : 'sequence<string>'         -> 'string'
      string<=5       : 'string<5>'                -> 'string<5>'
      string<=5[3]    : 'string<5>[3]'             -> 'string<5>'
      string<=5[<=10] : 'sequence<string<5>, 10>'  -> 'string<5>'
      string<=5[]     : 'sequence<string<5>>'      -> 'string<5>'
      string[<=10]    : 'sequence<string, 10>'     -> 'string'
    """
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
def bounded_string_size(field_type: str, check_valid: bool = True) -> int:
    """Maximum length of a string in the field, (-1 if not string)"""
    if check_valid and not is_bounded_string(field_type):
        return -1

    field_type = strip_array_from_field_type(field_type)
    start_ix = field_type.find(__BOUNDED_STRING_DELIM)
    if start_ix >= 0:
        start_ix += len(__BOUNDED_STRING_DELIM)
        end_ix = field_type.rfind('>')
        try:
            return int(field_type[start_ix:end_ix])
        except ValueError:
                pass
    return -1

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

def separate_field_from_array_information(field_name: str) -> Tuple[str, bool, int]:
    """
    Separates the mesage slot name from the index information

    eg:
        /positions[0]       -> /positions, True, 0
        /positions[0]/pos   -> /positions[0]/pos, False, -1
        /positions          -> /positions, False, -1

    If the input is malformed, (eg. `/positions[[0]`, we return '', False, -1 )
    :returns: the slot name, is_array, index
    """
    logger = __LOGGER.get_child(__name__)

    # Check for array index information
    if field_name[-1] != ']':
        return field_name, False, -1

    field_name_tokens = field_name.rsplit('[', 1)
    is_indexed = False
    index = -1
    if len(field_name_tokens) > 1:
        is_indexed = True
        try:
            index = int(field_name_tokens[1].rstrip(']'))
        except ValueError:
            logger.warn('Invalid field name: [%s]' % field_name)
            return "", False, -1

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

    :returns: slot class, array_info where array_info is a map
    :rtype: class, map
    """
    logger = __LOGGER.get_child(__name__)

    if is_primitive_type(message_class):
        logger.info("message_class: %s is primitive. Cannot get field information" % message_class)
        return None, None

    array_info = None
    slot_class_str = ""
    is_indexed = False
    index = -1


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
        logger.info("slot_path is empty, returning message_class info")

    else:
        # len(fields)> 0 indicates that we should traverse msg fields
        for field_name in fields:
            field_name, is_indexed, index = \
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
