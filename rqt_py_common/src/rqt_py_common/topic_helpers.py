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


def get_topic_names_and_types(node=None):
    """
    Get avaliable topic names and types.

    Note: in ROS2, only nodes can query the topic information
    @param node: a ROS node
    @type node: rclpy.node.Node or None

    If node is None, then this method will create a node, use it to get topic
    information and then destroy the node.
    """
    if node is not None:
        return node.get_topic_names_and_types()

    import rclpy
    shutdown_rclpy = False
    if not rclpy.ok():
        shutdown_rclpy = True
        rclpy.init()

    node = rclpy.create_node('TopicHelpers__get_topic_names_and_types')

    # Give the node time to learn about the graph
    rclpy.spin_once(node, timeout_sec=0.5)

    topic_list = node.get_topic_names_and_types()

    node.destroy_node()
    if shutdown_rclpy:
        rclpy.shutdown()

    return topic_list


_message_class_cache = {}  # noqa
def get_message_class(message_type):
    """
    get_message_class: gets the message class from a string representation.

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
        python_pkg = __import__('%s.%s' % (package, "msg"))
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


def get_type_class(type_name):
    """
    get_type_class: gets the python type from an idl string.

    See: https://github.com/ros2/design/blob/gh-pages/articles/142_idl.md

    @param type_name: the IDL type of field
    @type message_type: str
    """
    if type_name in ['float', 'float32', 'float64',
                     'double', 'long double']:
        return float

    elif type_name in ['char', 'wchar', 'string', 'wstring']:
        return str

    elif type_name in ['octet']:
        return bytes

    elif type_name in [
            'int8', 'uint8',
            'int16', 'uint16',
            'int32', 'uint32',
            'int64', 'uint64']:
        return int

    elif type_name in ['bool', 'boolean']:
        return bool

    else:
        return None


def get_field_types(topic_name):
    """
    Get the Python type of a specific field in the given registered topic.

    If the field is an array, the type of the array's values are returned and the is_array flag is
    set to True. This is a static type check, so it works for unpublished topics and with empty
    arrays.

    :param topic_name: name of field of a registered topic, ``str``, i.e. '/rosout/file'
    :returns: field_type, is_array
    """
    # Note: Mlautman 11/2/18
    #       In ROS2 multiple msg types can be used with a single topic making this
    #       funciton a bad candidate to port to ROS2
    logger = logging.get_logger("topic_helpers")
    logger.error("get_field_type is not implemented in ROS2")
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
    :param slot_path: path to the slot inside the message class, ``str``, i.e. '_header/_seq'
    :returns: field_type, is_array
    """
    is_array = False
    fields = [f for f in slot_path.split('/') if f]
    for field_name in fields:
        slot_class_name = message_class._slot_types[message_class.__slots__.index(field_name)]
        is_array = slot_class_name.find('[') >= 0
        message_class = get_message_class(slot_class_name[:slot_class_name.find('[')])

    return message_class, is_array
