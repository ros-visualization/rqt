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

from typing import Mapping, Tuple, Optional

from rclpy import logging

from python_qt_binding.QtCore import qWarning

from rqt_py_common.message_helpers import get_message_class

from rqt_py_common import message_field_type_helpers


__LOGGER = logging.get_logger('topic_helpers')

def is_primitive_type(field_type):
    """
    Checks if the field type is a primitive type

    Input can be either str or an instance of the field type

    If the field type is a string return if it is in PRIMITIVE_TYPES
    If the field type is a class, then we check if it is generated msg type
        or not.

    """
    __LOGGER.get_child("is_primitive_type").warn(
        "[[DEPRECATED]] Please use message_field_type_helpers.is_primitive_type")
    return message_field_type_helpers.is_primitive_type(field_type)

def get_type_class(field_type):
    __LOGGER.get_child('get_type_class').warn(
        '[[DEPRECATED]] Please use message_field_type_helpers.class_is_primitive_type or '
        'message_field_type_helpers.get_primitive_python_class')
    if type(field_type) is str:
        return message_field_type_helpers.get_primitive_python_class(field_type)
    else:
        return message_field_type_helpers.class_is_primitive_type(field_type)

def get_field_type(path_to_target, node):
    """
    Get the Python type of a specific field in the given registered topic.

    If the field is an array, the type of the array's values are returned and the is_array flag
    is set to True

    :param path_to_target: name of field of a registered topic,
        eg. '/ns/node/topic/field_a' or '/ns/node/topic'
    :type path_to_target: str

    :param node: a rclpy.Node

    :rtype: Tuple[Class, bool]
    :returns: field_type, is_array
    """
    topic_names_and_types = node.get_topic_names_and_types()
    return _get_field_type(topic_names_and_types, path_to_target)


def _get_field_type(topic_names_and_types, path_to_target):  # noqa: C901
    """Testable helper function for get_field_type."""
    logger = __LOGGER.get_child(__name__)
    delim = '/'
    tokenized_target = path_to_target.strip(delim).split(delim)
    for topic_name, types in topic_names_and_types:
        tokenized_topic_name = topic_name.strip(delim).split(delim)
        # If the path_to_target starts with the topic we have found a potential match
        if tokenized_target[:len(tokenized_topic_name)] == tokenized_topic_name:
            # If the path_to_target passed in was the topic address not the address of a field
            if tokenized_target == tokenized_topic_name:
                # If there is more than one type of topic on path_to_target
                if len(types) > 1:
                    logger.warn(
                        'Ambiguous request. Multiple topic types found on: {}'.format(path_to_target))
                    return None, False
                # If the types array is empty then something weird has happend
                if len(types) == 0:
                    logger.warn(
                        'No msg types found on: {}'.format(path_to_target))
                    return None, False

                # If there is only one msg type
                msg_class = get_message_class(types[0])
                return msg_class, False

            else:
                # The topic must be a substring of the path_to_target
                # Get the address of the field in the messgage class
                field_address = path_to_target[len(topic_name):]

                # Iterate through the message types on the given topic and see if any match the
                # path that was provided
                for msg_type_str in types:
                    try:
                        msg_class = get_message_class(msg_type_str)
                        field_type, field_info = \
                            message_field_type_helpers.get_slot_class_and_field_information(
                                msg_class, field_address
                            )
                        if field_type:
                            return field_type, field_info.is_array
                    except KeyError:
                        pass
    logger.debug('faild to find field type: {}'.format(path_to_target))
    return None, False

def slot_is_array(field_type) -> bool:
    """
    Returns if the field type is an array.

    :param field_type: Field type as produced by get_fields_and_field_types()
    :type field_type: str

    :rtype: bool
    """
    __LOGGER.get_child('slot_is_array').warn(
        '[[DEPRECATED]] Please use message_field_type_helpers.is_array')
    return message_field_type_helpers.is_array

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
    logger = __LOGGER.get_child('get_slot_type')
    logger.warn(
        '[get_slot_type] deprecated. Please use: get_slot_class_and_field_information')

    slot_class, field_info = \
        message_field_type_helpers.get_slot_class_and_field_information(message_class, slot_path)
    if field_info is None:
        logger.warn('get_slot_type could not parse slot_path for msg class')
        logger.warn('\t slot_path: %s' % slot_path)
        logger.warn('\t message_class: %s' % message_class)
        return None, False

    return slot_class, field_info.is_array

def separate_field_from_array_information(slot_path: str) -> Tuple[str, bool, Optional[int]]:
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
    if slot_path[-1] != ']':
        return slot_path, is_indexed, index

    slot_path_tokens = slot_path.rsplit('[', 1)
    if len(slot_path_tokens) > 1:
        is_indexed = True
        try:
            index = int(slot_path_tokens[1].rstrip(']'))
        except ValueError:
            return "", False, None

    return slot_path_tokens[0], is_indexed, index

