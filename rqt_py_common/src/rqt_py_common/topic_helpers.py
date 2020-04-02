# Copyright (c) 2011, Dorian Scholz, TU Darmstadt
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

from rclpy import logging

from rqt_py_common.message_helpers import get_message_class

SEQUENCE_DELIM = 'sequence<'
PRIMITIVE_TYPES = [
    'int8', 'uint8',
    'int16', 'uint16',
    'int32', 'uint32',
    'int64', 'uint64',
    'float', 'float32', 'float64',
    'double', 'long double',
    'char', 'wchar',
    'octet',
    'string',
    'bool', 'boolean',
    # deprecated in ros1:
    'char', 'byte']


def is_primitive_type(type_str):
    # Note: this list a combination of primitive types from ROS1 and the new IDL definitions
    return type_str in PRIMITIVE_TYPES


def get_type_class(type_name):
    """
    Gets the python type from an idl string.

    See: https://github.com/ros2/design/blob/gh-pages/articles/142_idl.md

    @param type_name: the IDL type of field
    @type message_type: str
    """
    # Note: this list a combination of primitive types from ROS1 and the new IDL definitions

    if type_name in ['float', 'float32', 'float64',
                     'double', 'long double']:
        return float

    # TODO(mlautman): char should be a string of length one. We do not currently support this
    if type_name in ['char', 'wchar', 'string', 'wstring']:
        return str

    if type_name in ['octet']:
        return bytes

    if type_name in [
            'int8', 'uint8',
            'int16', 'uint16',
            'int32', 'uint32',
            'int64', 'uint64']:
        return int

    if type_name in ['bool', 'boolean']:
        return bool

    return None


def get_field_type(target, node):
    """
    Get the Python type of a specific field in the given registered topic.

    If the field is an array, the type of the array's values are returned and the is_array flag
    is set to True

    :param target: name of field of a registered topic, ``str``, i.e. '/rosout/file'
    :param node: a rclpy.Node
    :returns: field_type, is_array
    """
    topic_names_and_types = node.get_topic_names_and_types()
    return _get_field_type(topic_names_and_types, target)


def _get_field_type(topic_names_and_types, target):  # noqa: C901
    """Testable helper function for get_field_type."""
    logger = logging.get_logger('topic_helpers._get_field_type')
    delim = '/'
    tokenized_target = target.strip(delim).split(delim)
    for topic_name, types in topic_names_and_types:
        tokenized_topic_name = topic_name.strip(delim).split(delim)
        # If the target starts with the topic we have found a potential match
        if tokenized_target[:len(tokenized_topic_name)] == tokenized_topic_name:
            # If the target passed in was the topic address not the address of a field
            if tokenized_target == tokenized_topic_name:
                # If there is more than one type of topic on target
                if len(types) > 1:
                    logger.warn(
                        'Ambiguous request. Multiple topic types found on: {}'.format(target))
                    return None, False
                # If the types array is empty then something weird has happend
                if len(types) == 0:
                    logger.warn(
                        'No msg types found on: {}'.format(target))
                    return None, False

                # If there is only one msg type
                msg_class = get_message_class(types[0])
                return msg_class, False

            else:
                # The topic must be a substring of the target
                # Get the address of the field in the messgage class
                field_address = target[len(topic_name):]

                # Iterate through the message types on the given topic and see if any match the
                # path that was provided
                for msg_type_str in types:
                    try:
                        msg_class = get_message_class(msg_type_str)
                        field_type, is_array = get_slot_type(msg_class, field_address)
                        if field_type:
                            return field_type, is_array
                    except KeyError:
                        pass
    logger.debug('faild to find field type: {}'.format(target))
    return None, False


def slot_is_array(field_type) -> bool:
    """
    Returns if the field type is an array.

    :param field_type: Field type as produced by get_fields_and_field_types()
    :type field_type: str

    :rtype: bool
    """
    return field_type.startswith(SEQUENCE_DELIM) or \
        field_type.find('[') >= 0


def strip_array_from_field_class_str(field_class_str):
    """
    Removes the array information leaving just the slot class str

    Parse items such as (from rosidl_generator_py definitions):
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
        string[3]       -> 'string[3]'                -> List[str]
        string[]        -> 'sequence<string>'         -> List[str]
        string<=5[3]    -> 'string<5>[3]'             -> List[str]
        string<=5[<=10] -> 'sequence<string<5>, 10>'  -> List[str]
        string<=5[]     -> 'sequence<string<5>>'      -> List[str]
        string[<=10]    -> 'sequence<string, 10>'     -> List[str]

    :param field_class_str: The field_class_str such as one as created by
        get_fields_and_field_types()
    :type field_class_str: str

    :returns: the field_type of with the list component stripped out
    :rtype str: str
    """
    if field_class_str.startswith(SEQUENCE_DELIM):
        field_class_str = field_class_str[len(SEQUENCE_DELIM):]

    end_of_base_type_delim = ['<', '[', ',', '>']
    for delim in end_of_base_type_delim:
        delim_ix = field_class_str.find(delim)
        if delim_ix >= 0:
            field_class_str = field_class_str[:delim_ix]

    return field_class_str


def remove_sequence_from_type_str(type_str):
    if type_str.startswith(SEQUENCE_DELIM):
        # Check for bound sequence
        seq_start_ix = len(SEQUENCE_DELIM)
        seq_end_ix = type_str.rfind(',')
        # If not bound check for unbound sequence
        if seq_end_ix < 0:
            seq_end_ix = type_str.rfind('>')

        # If we have found an end to the sequence, then we strip that
        if seq_end_ix > 0:
            type_str = type_str[seq_start_ix:seq_end_ix]

    return type_str

def get_array_information(type_str):
    return {
        "type_string": strip_array_from_field_class_str(type_str),
        "is_array": slot_is_array(type_str),
        "is_static_array": is_static_array(type_str),
        "static_array_size": get_static_array_size(type_str),
        "is_bounded_array": is_bounded_array(type_str),
        "bounded_array_size": get_bounded_array_size(type_str),
        "is_unbounded_array": is_unbounded_array(type_str),
        "is_bounded_string": is_bounded_string(type_str),
        "bounded_string_size": get_bounded_string_size(type_str)
    }


def is_bounded_array(type_str):
    return type_str.startswith(SEQUENCE_DELIM) and type_str.find(',') >= 0


def is_unbounded_array(type_str):
    return type_str.startswith(SEQUENCE_DELIM) and type_str.find(',') < 0


def is_static_array(type_str):
    return type_str.find('[') >= 0


def get_static_array_size(type_str):
    start_ix = type_str.find('[')
    end_ix = type_str.find(']')
    try:
        return int(type_str[start_ix + 1:end_ix])
    except ValueError:
        return -1

def get_bounded_array_size(type_str):
    bounded_size_start_ix = type_str.find(', ')
    try:
        return int(type_str[bounded_size_start_ix + 2:-1])
    except ValueError:
        return -1

def is_bounded_string(type_str):
    bounded_string_delim = 'string<'
    return type_str.find(bounded_string_delim) >= 0


def get_bounded_string_size(type_str):
    bounded_string_delim = 'string<'
    type_str = remove_sequence_from_type_str(type_str)
    start_ix = type_str.find(bounded_string_delim)
    if start_ix >= 0:
        start_ix += len(bounded_string_delim)
        end_ix = type_str.rfind('>')
        try:
            return int(type_str[start_ix:end_ix])
        except ValueError:
            return -1

def get_slot_class(slot_class_string):
    if is_primitive_type(slot_class_string):
        return get_type_class(slot_class_string)
    else:
        return get_message_class(slot_class_string)


def get_slot_type_str(message_class, slot_path):
    """
    Get the Python type as a string of a slot in the given message class.

    If the field is an array, the type of the array's values are returned and the is_array rval is
    set to True. This is a static type check, so it works for unpublished topics and with empty
    arrays.

    :param message_class: message class type as str
    :param slot_path: path to the slot inside the message class, ``str``, i.e. '_header/_seq'

    :returns: field_type as str, is_array
    :rtype: str, bool
    """
    _, slot_class_str, is_array = get_slot_class_and_str(message_class, slot_path)
    return slot_class_str, is_array


def get_slot_type(message_class, slot_path):
    """
    Get the Python type of a specific slot in the given message class.

    If the field is an array, the type of the array's values are returned and the is_array flag is
    set to True. This is a static type check, so it works for unpublished topics and with empty
    arrays.

    :param message_class: message class type as a class
    :param slot_path: path to the slot inside the message class, ``str``, i.e. '_header/_seq'

    :returns: field_type, is_array
    :rtype: str, bool
    """
    slot_class, _, is_array = get_slot_class_and_str(message_class, slot_path)
    return slot_class, is_array


def get_slot_class_and_str(message_class, slot_path):
    """
    Get the Python type of a specific slot in the given message class.

    If the field is an array, the type of the array's values are returned and the is_array flag is
    set to True. This is a static type check, so it works for unpublished topics and with empty
    arrays.

    :param message_class: message class type as a class
    :param slot_path: path to the slot inside the message class, ``str``, i.e. '_header/_seq'

    :returns: slot class, slot string rep, is_array
    :rtype: class, str, bool
    """
    slot_class_str = None
    is_array = False

    fields = [f for f in slot_path.split('/') if f]
    for field_name in fields:
        is_array = False
        slot_class_str = None
        message_class_slots = message_class.get_fields_and_field_types()
        if not field_name in message_class_slots:
            return None, None, is_array

        slot_class_str = message_class_slots[field_name]
        if slot_is_array(slot_class_str):
            slot_class_str = strip_array_from_field_class_str(slot_class_str)
            is_array = True

        message_class = get_slot_class(slot_class_str)

    return message_class, slot_class_str, is_array
