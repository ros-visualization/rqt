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

import rclpy

from rclpy import logging
from ament_index_python import get_resource


def get_topic_names_and_types():
    shutdown_rclpy = False
    if not rclpy.ok():
        shutdown_rclpy = True
        rclpy.init()

    node = rclpy.create_node("TopicHelpers__get_topic_names_and_types")
    # Give the node time to learn about the graph
    rclpy.spin_once(node, timeout_sec=0.05)

    topic_list = node.get_topic_names_and_types()

    node.destroy_node()
    if shutdown_rclpy:
        rclpy.shutdown()

    return topic_list


_logger = logging.get_logger("topic_helpers")
_message_class_cache = {}
def get_message_class(message_type):
    if message_type in _message_class_cache:
        return _message_class_cache[message_type]

    message_info = message_type.split("/")
    if len(message_info) == 2:
        package = message_info[0]
        base_type = message_info[1]
    elif len(message_info) == 1:
        package = "std_msgs"
        base_type = message_info[0]
    else:
        _logger.error(
            "Malformed message_type passed into get_message_class: {}".format(
                message_type))
        return None

    _, resource_path = get_resource('rosidl_interfaces', package)
    python_pkg = class_val = None
    try:
        # import the package
        python_pkg = __import__('%s.%s' % (package, "msg"))
    except ImportError:
        _logger.error("Failed to get message class: {}".format(message_type))

    if python_pkg:
        try:
            class_val = getattr(getattr(python_pkg, "msg"), base_type)
        except AttributeError:
            _logger.error("Failed to get message class: {}".format(message_type))

    if class_val:
        _message_class_cache[message_type] = class_val

    return class_val


def get_type_class(type_name):
    # if roslib.msgs.is_valid_constant_type(type_name):
    #     if type_name == 'string':
    #         return str
    #     elif type_name == 'bool':
    #         return bool
    #     else:
    #         return type(roslib.msgs._convert_val(type_name, 0))
    pass

def get_field_type(topic_name):
    """
    Get the Python type of a specific field in the given registered topic.

    If the field is an array, the type of the array's values are returned and the is_array flag is
    set to True. This is a static type check, so it works for unpublished topics and with empty
    arrays.

    :param topic_name: name of field of a registered topic, ``str``, i.e. '/rosout/file'
    :returns: field_type, is_array
    """
    # get topic_type and message_evaluator
    # topic_type, real_topic_name, _ = get_topic_type(topic_name)
    # if topic_type is None:
    #     # qDebug('topic_helpers.get_field_type(%s): get_topic_type failed' % (topic_name))
    #     return None, False

    # message_class = roslib.message.get_message_class(topic_type)
    # if message_class is None:
    #     qDebug('topic_helpers.get_field_type(%s): get_message_class(%s) failed' %
    #            (topic_name, topic_type))
    #     return None, False

    # slot_path = topic_name[len(real_topic_name):]
    # return get_slot_type(message_class, slot_path)
    pass

def get_slot_type(message_class, slot_path):
    """
    Get the Python type of a specific slot in the given message class.

    If the field is an array, the type of the array's values are returned and the is_array flag is
    set to True. This is a static type check, so it works for unpublished topics and with empty
    arrays.

    :param message_class: message class type, ``type``, usually inherits from genpy.message.Message
    :param slot_path: path to the slot inside the message class, ``str``, i.e. 'header/seq'
    :returns: field_type, is_array
    """
    # is_array = False
    # fields = [f for f in slot_path.split('/') if f]
    # for field_name in fields:
    #     try:
    #         field_name, _, field_index = roslib.msgs.parse_type(field_name)
    #     except roslib.msgs.MsgSpecException:
    #         return None, False
    #     if field_name not in getattr(message_class, '__slots__', []):
    #         # qDebug('topic_helpers.get_slot_type(%s, %s): field not found: %s' %
    #         # (message_class, slot_path, field_name))
    #         return None, False
    #     slot_type = message_class._slot_types[message_class.__slots__.index(field_name)]
    #     slot_type, slot_is_array, _ = roslib.msgs.parse_type(slot_type)
    #     is_array = slot_is_array and field_index is None

    #     message_class = get_type_class(slot_type)
    # return message_class, is_array
    pass


def is_slot_numeric(topic_name):
    """
    Check is a slot in the given topic is numeric, or an array of numeric values.

    This is a static type check, so it works for unpublished topics and with empty arrays.

    :param topic_name: name of field of a registered topic, ``str``, i.e. '/rosout/file'
    :returns: is_numeric, is_array, description
    """
    # field_type, is_array = get_field_type(topic_name)
    # if field_type in (int, float):
    #     if is_array:
    #         message = 'topic "%s" is numeric array: %s[]' % (topic_name, field_type)
    #     else:
    #         message = 'topic "%s" is numeric: %s' % (topic_name, field_type)
    #     return True, is_array, message

    # return False, is_array, 'topic "%s" is NOT numeric: %s' % (topic_name, field_type)
    pass


def find_slots_by_type_dfs(msg_class, slot_type):
    """
    Search inside msg_class for all slots of type slot_type and return their paths.

    Uses a depth first search.

    :param msg_class: The class to search in.
    :param slot_type: The type name or class to search for (e.g. 'float64' or Quaternion).
    :return: List of paths to slots of type slot_type inside msg_class (e.g. ['header/frame_id']).
    """
    # def _find_slots(msg_class, slot_type):
    #     paths = []
    #     if msg_class == slot_type:
    #         paths.append([])
    #         return paths

    #     for slot_name, slot_type_name in zip(msg_class.__slots__, msg_class._slot_types):
    #         slot_type_name, is_array, _ = roslib.msgs.parse_type(slot_type_name)
    #         if is_array:
    #             slot_name += '[]'
    #         if roslib.msgs.is_valid_constant_type(slot_type_name):
    #             if slot_type_name == slot_type:
    #                 paths.append([slot_name])
    #             continue

    #         slot_class = roslib.message.get_message_class(slot_type_name)
    #         if slot_class is not None:
    #             inner_paths = _find_slots(slot_class, slot_type)
    #             paths.extend([[slot_name] + path for path in inner_paths])

    #     return paths

    # return ['/'.join(path) for path in _find_slots(msg_class, slot_type)]
    pass

def find_slots_by_type_bfs(msg_class, slot_type):
    """
    Search inside msg_class for all slots of type slot_type and return their paths.

    Uses a breadth first search, so it will find the most shallow matches first.

    :param msg_class: The class to search in.
    :param slot_type: The type name or class to search for (e.g. 'float64' or Quaternion).
    :return: List of paths to slots of type slot_type inside msg_class (e.g. ['header/frame_id']).
    """
    # paths = []
    # queue = [(msg_class, [])]
    # while queue:
    #     msg_class, path = queue.pop(0)
    #     if msg_class == slot_type:
    #         paths.append(path)
    #         continue

    #     for slot_name, slot_type_name in zip(msg_class.__slots__, msg_class._slot_types):
    #         slot_type_name, is_array, _ = roslib.msgs.parse_type(slot_type_name)
    #         if is_array:
    #             slot_name += '[]'
    #         if roslib.msgs.is_valid_constant_type(slot_type_name):
    #             if slot_type_name == slot_type:
    #                 paths.append(path + [slot_name])
    #             continue

    #         slot_class = roslib.message.get_message_class(slot_type_name)
    #         if slot_class is not None:
    #             queue.append((slot_class, path + [slot_name]))

    # return ['/'.join(path) for path in paths]
    pass
