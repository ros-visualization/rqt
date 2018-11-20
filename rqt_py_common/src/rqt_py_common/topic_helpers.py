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

from ament_index_python import get_resource

from rclpy import logging


_message_class_cache = {}


def get_message_class(message_type):
    """
    Gets the message class from a string representation.

    @param message_type: the type of message in the form `msg_pkg/Message`
    @type message_type: str
    """
    logger = logging.get_logger('get_message_class')
    if message_type in _message_class_cache:
        return _message_class_cache[message_type]

    message_info = message_type.split('/')
    if len(message_info) == 2:
        package = message_info[0]
        base_type = message_info[1]
    elif len(message_info) == 1:
        package = 'std_msgs'
        base_type = message_info[0]
    else:
        logger.error(
            'Malformed message_type passed into get_message_class: {}'.format(
                message_type))
        return None

    _, resource_path = get_resource('rosidl_interfaces', package)
    python_pkg = class_val = None
    try:
        # import the package
        python_pkg = __import__('%s.%s' % (package, 'msg'))
    except ImportError:
        logger.error('Failed to get message class: {}'.format(message_type))

    if python_pkg:
        try:
            class_val = getattr(getattr(python_pkg, 'msg'), base_type)
        except AttributeError:
            if len(base_type):
                base_type = ''.join([base_type[0].upper(), base_type[1:]])

        if not class_val:
            try:
                class_val = getattr(getattr(python_pkg, 'msg'), base_type)
            except AttributeError:
                logger.error('Failed to get message class: {}'.format(message_type))

    if class_val:
        _message_class_cache[message_type] = class_val

    return class_val


def _is_primative_type(type_str):
    # Note: this list a combination of primitive types from ROS1 and the new IDL definitions
    primitive_types = [
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
    return type_str in primitive_types


def get_type_class(type_name):
    """
    Gets the python type from an idl string.

    See: https://github.com/ros2/design/blob/gh-pages/articles/142_idl.md

    @param type_name: the IDL type of field
    @type message_type: str
    """
    # Note: this list a combination of primitive types from ROS1 and the new IDL definitions

    if not _is_primative_type(type_name.lower()):
        return get_message_class(type_name)

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
                        return field_type, is_array
                    except KeyError:
                        pass

    logger.debug('faild to find field type: {}'.format(target))
    return None, False


def get_slot_type(message_class, slot_path):
    """
    Get the Python type of a specific slot in the given message class.

    If the field is an array, the type of the array's values are returned and the is_array flag is
    set to True. This is a static type check, so it works for unpublished topics and with empty
    arrays.

    :param message_class: message class type, ``type``, usually inherits from genpy.message.Message
    :param slot_path: path to the slot inside the message class, ``str``, i.e. '_header/_seq'
    :returns: field_type, is_array
    """
    is_array = False
    fields = [f for f in slot_path.split('/') if f]
    for field_name in fields:
        slot_class_name = message_class.get_fields_and_field_types()[field_name]

        array_index = slot_class_name.find('[')
        if array_index >= 0:
            is_array = True
            message_class = get_type_class(slot_class_name[:array_index])
        else:
            is_array = False
            message_class = get_type_class(slot_class_name)

    return message_class, is_array
