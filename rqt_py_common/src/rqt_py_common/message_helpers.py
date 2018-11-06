# Copyright (c) 2018, PickNik Robotics
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
#   * Neither the name of the PickNik Robotics nor the names of its
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
from ament_index_python import get_resources
from ament_index_python import has_resource

from rclpy import logging

MSG_MODE = 'msg'
SRV_MODE = 'srv'
ACTION_MODE = 'action'

# Taken from https://github.com/ros2/ros2cli/blob/bouncy/ros2msg/ros2msg/api/__init__.py
def get_all_service_types():
    all_service_types = {}
    for package_name in get_resources('rosidl_interfaces').keys():
        service_types = get_service_types(package_name)
        if service_types:
            all_service_types[package_name] = service_types
    return all_service_types


def get_service_types(package_name):
    if not has_resource('packages', package_name):
        raise LookupError('Unknown package name')
    try:
        content, _ = get_resource('rosidl_interfaces', package_name)
    except LookupError:
        return []
    interface_names = content.splitlines()
    # TODO(dirk-thomas) this logic should come from a rosidl related package
    # Only return services in srv folder
    all_srvs = [
        n[4:-4] for n in interface_names if n.startswith('srv/') and n.endswith('.srv')]
    all_srvs.extend(
        [n[:-4] for n in interface_names if not n.startswith('srv/') and n.endswith('.srv')])
    return all_srvs


def get_all_message_types():
    all_message_types = {}
    for package_name in get_resources('rosidl_interfaces').keys():
        message_types = get_message_types(package_name)
        if message_types:
            all_message_types[package_name] = message_types
    return all_message_types


def get_message_types(package_name):
    if not has_resource('packages', package_name):
        raise LookupError('Unknown package name')
    try:
        content, _ = get_resource('rosidl_interfaces', package_name)
    except LookupError:
        return []
    interface_names = content.splitlines()
    # TODO(dirk-thomas) this logic should come from a rosidl related package
    # Only return messages in msg folder
    all_msgs = [
        n[4:-4] for n in interface_names if n.startswith('msg/') and n.endswith('.msg')]
    all_msgs.extend(
        [n[:-4] for n in interface_names if not n.startswith('msg/') and n.endswith('.msg')])
    return all_msgs


def _get_message_service_class_helper(message_type, mode, logger):
    if not (mode == MSG_MODE or mode == SRV_MODE):
        logger.warn('invalid mode {}'.format(mode))
        return None

    message_info = message_type.split('/')
    if len(message_info) == 2:
        package = message_info[0]
        base_type = message_info[1]
    elif len(message_info) == 1:
        package = 'std_' + mode + 's'
        base_type = message_info[0]
    else:
        logger.error(
            'Malformed message_type: {}'.format(message_type))
        return None

    _, resource_path = get_resource('rosidl_interfaces', package)
    python_pkg = class_val = None
    try:
        # import the package
        python_pkg = __import__('%s.%s' % (package, mode))
    except ImportError:
        logger.error('Failed to import class: {} as {}.{}'.format(message_type, package, mode))

    if python_pkg:
        try:
            class_val = getattr(getattr(python_pkg, mode), base_type)
        except AttributeError:
            if len(base_type):
                # Try the same thing but with the first letter capitalized
                base_type = ''.join([base_type[0].upper(), base_type[1:]])

        if not class_val:
            try:
                class_val = getattr(getattr(python_pkg, mode), base_type)
            except AttributeError:
                logger.error('Failed to load class: {}'.format(message_type))

    return class_val


_srv_class_cache = {}


def get_service_class(srv_type):
    """
    Gets the service class from a string representation.

    @param srv_type: the type of service in the form `srv_pkg/Service`
    @type srv_type: str
    """
    if srv_type in _srv_class_cache:
        return _srv_class_cache[srv_type]

    logger = logging.get_logger('get_service_class')
    class_val = _get_message_service_class_helper(srv_type, SRV_MODE, logger)

    if class_val is not None:
        _srv_class_cache[srv_type] = class_val

    return class_val


_message_class_cache = {}


def get_message_class(message_type):
    """
    Gets the message class from a string representation.

    @param message_type: the type of message in the form `msg_pkg/Message`
    @type message_type: str
    """
    if message_type in _message_class_cache:
        return _message_class_cache[message_type]

    logger = logging.get_logger('get_message_class')

    class_val = _get_message_service_class_helper(message_type, MSG_MODE, logger)

    if class_val is not None:
        _message_class_cache[message_type] = class_val

    return class_val


def get_message_text_from_class(msg_class):
    msg_slot_dict = msg_class.get_fields_and_field_types()
    return ''.join(
        ['{0:{width}}{1}\n'.format(slot_type, slot_name, width=30) for
            slot_name, slot_type in msg_slot_dict.items()])


def get_service_text_from_class(srv_class):
    srv_slot_dict_req = srv_class.Request.get_fields_and_field_types()
    srv_slot_dict_res = srv_class.Response.get_fields_and_field_types()
    srv_txt = [
        '{0:{width}}{1}\n'.format(slot_type, slot_name, width=30) for
        slot_name, slot_type in srv_slot_dict_req.items()]
    srv_txt.extend(['-----\n'])
    srv_txt.extend([
        '{0:{width}}{1}\n'.format(slot_type, slot_name, width=30) for
        slot_name, slot_type in srv_slot_dict_res.items()])
    return ''.join(srv_txt)
