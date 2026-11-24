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
import functools
import importlib
import os

from ament_index_python import get_package_share_directory
from ament_index_python import get_resource
from ament_index_python import get_resources

from rclpy import logging

MSG_MODE = 'msg'
SRV_MODE = 'srv'
ACTION_MODE = 'action'

ROSIDL_FILTERS = {
    MSG_MODE: lambda n: n.startswith('msg/') and n.endswith('.msg'),
    SRV_MODE: lambda n: n.startswith('srv/') and n.endswith('.srv'),
    ACTION_MODE: lambda n: n.startswith('action/') and n.endswith('.idl')
}


def _strip_type_name(type_name):
    prefixes = ['msg/', 'srv/', 'action/']
    suffixes = ['.msg', '.srv', '.idl']
    for prefix, suffix in zip(prefixes, suffixes):
        found_match = False
        if type_name.startswith(prefix):
            type_name = type_name[len(prefix):]
            found_match = True
        if type_name.endswith(suffix):
            type_name = type_name[:len(type_name) - len(suffix)]
            found_match = True
        if found_match:
            break
    return type_name


def _filter_rosidl_types(rosidl_type, interface_names):
    """
    Filter a list of rosidl interfaces with a specific type.

    :param rosidl_type: rosidl type to filter by, one of ['msg', 'srv', 'action']
    :type rosidl_type: str
    :param interface_names: list of interface names return from ament_index_python.get_resource
    :type interface_names: list(str)
    :returns: a list of the filtered rosidl types
    """
    if rosidl_type not in ROSIDL_FILTERS:
        raise ValueError(
            f'Invalid rosidl_type type "{rosidl_type}". '
            f'Needs to be one of {ROSIDL_FILTERS.keys()}')

    filter_fn = ROSIDL_FILTERS[rosidl_type]
    filtered = filter(filter_fn, interface_names)
    stripped = map(_strip_type_name, filtered)
    return list(stripped)


def get_rosidl_types_of_type(package_name, rosidl_type):
    """
    Retrieve a list of rosidl interfaces of a specific type provided by a ROS2 package.

    :param package_name: ROS2 package name
    :type package_name: str
    :param rosidl_type: rosidl type to filter by, one of ['msg', 'srv', 'action']
    :type rosidl_type: str
    :returns: a list of the filtered rosidl types
    """
    try:
        content, _ = get_resource('rosidl_interfaces', package_name)
    except LookupError:
        return []
    interface_names = content.splitlines()
    return _filter_rosidl_types(rosidl_type, interface_names)


def get_rosidl_types(package_name):
    """
    Retrieve a dictionary of lists for all rosidl interface types provided by a ROS2 package.

    :param package_name: ROS2 package name
    :type package_name: str
    :returns: a dictionary mapping rosidl type to a list {'msg': [...], 'srv': ... }
    """
    rosidl_types = {}
    try:
        content, _ = get_resource('rosidl_interfaces', package_name)
    except LookupError:
        return {}
    interface_names = content.splitlines()
    for filter_type in ROSIDL_FILTERS.keys():
        rosidl_types[filter_type] = _filter_rosidl_types(filter_type, interface_names)
    return rosidl_types


def get_all_rosidl_types():
    """
    Retrieve a dictionary mapping packages to their provided rosidl interfaces.

    :returns: a dictionary mapping packages to their types
    """
    all_rosidl_types = {}
    for package_name in get_resources('rosidl_interfaces').keys():
        all_rosidl_types[package_name] = get_rosidl_types(package_name)
    return all_rosidl_types


def get_all_rosidl_types_of_type(rosidl_type):
    """
    Retrieve a dictionary mapping packages to a specific rosidl type.

    :returns: a dictionary mapping packages to the rosidl interfaces of a specific type
    """
    if rosidl_type not in ROSIDL_FILTERS:
        raise ValueError(
            f'Invalid rosidl_type type {rosidl_type}. '
            f'Needs to be one of {ROSIDL_FILTERS.keys()}')

    all_rosidl_types = get_all_rosidl_types()
    return {
        package_name:
            d[rosidl_type] for package_name, d in all_rosidl_types.items() if d[rosidl_type]
    }


def get_all_service_types():
    """
    Use the ament index to iterate through packages and calls get_service_types on each one.

    :returns: a dictionary of the form {'package_name', ['srv1', 'srv2', ...]}
    """
    return get_all_rosidl_types_of_type(SRV_MODE)


def get_service_types(package_name):
    """
    Use the ament index gind all services avialable in the package.

    :param package_name: a string eg 'std_srvs'
    :returns: a dictionary of the form {'package_name', ['srv1', 'srv2', ...]}
    """
    return get_rosidl_types_of_type(package_name, SRV_MODE)


def get_all_message_types():
    """
    Use the ament index to iterate through packages and calls get_message_types on each one.

    :returns: a dictionary of the form {'package_name', ['msg1', 'msg2', ...]}
    """
    return get_all_rosidl_types_of_type(MSG_MODE)


def get_message_types(package_name):
    """
    Use the ament index to find all messages avialable in the package.

    :param package_name: a string eg 'std_msgs'
    :returns: a dictionary of the form {'std_msgs', ['Bool', 'String', ...]}
    """
    return get_rosidl_types_of_type(package_name, MSG_MODE)


def get_all_action_types():
    """
    Get all the packages that support actions and their action types.

    :returns: a dictionary of the form {'package_name', ['action1', 'action2', ...]}
    """
    return get_all_rosidl_types_of_type(ACTION_MODE)


def get_action_types(package_name):
    """
    Find all actions available in the package.

    :param package_name: a string eg 'test_msgs'
    :returns: a list of action names ['Fibonacci', ...]
    """
    return get_rosidl_types_of_type(package_name, ACTION_MODE)


def _get_rosidl_class_helper(message_type, mode, logger=None):  # noqa: C901
    """
    Logic to be used by get_message_class and get_service_class.

    :param message_type: name of the message or service class in the form
      'package_name/MessageName' or 'package_name/msg/MessageName'
    :type message_type: str
    :param mode: one of MSG_MODE, SRV_MODE or ACTION_MODE
    :type mode: str
    :param logger: The logger to be used for warnings and info
    :type logger: either rclpy.impl.rcutils_logger.RcutilsLogger or None

    :returns: The message or service class or None
    """
    if logger is None:
        logger = logging.get_logger('_get_message_service_class_helper')

    if mode not in ROSIDL_FILTERS.keys():
        logger.warning(f'invalid mode {mode}')
        return None

    message_info = message_type.split('/')
    if len(message_info) not in (2, 3):
        logger.error(f'Malformed message_type: {message_type}')
        return None
    if len(message_info) == 3 and message_info[1] != mode:
        logger.error(f'Malformed {mode} message_type: {message_type}')
        return None

    package = message_info[0]
    base_type = message_info[-1]

    try:
        _, resource_path = get_resource('rosidl_interfaces', package)
    except LookupError:
        return None
    python_pkg = None
    class_val = None

    try:
        # import the package
        python_pkg = importlib.import_module(f'{package}.{mode}')
    except ImportError:
        logger.info(f'Failed to import class: {message_type} as {package}.{mode}')
        return None

    try:
        class_val = getattr(python_pkg, base_type)
        return class_val

    except AttributeError:
        logger.info(f'Failed to load class: {message_type}')
        return None


_srv_class_cache = {}


def get_service_class(srv_type):
    """
    Get the service class from a string representation.

    :param srv_type: the type of service in the form
      `package_name/ServiceName` or `package_name/srv/ServiceName`
    :type srv_type: str

    :returns: None or the Class
    """
    if srv_type in _srv_class_cache:
        return _srv_class_cache[srv_type]

    logger = logging.get_logger('get_service_class')
    class_val = _get_rosidl_class_helper(srv_type, SRV_MODE, logger)

    if class_val is not None:
        _srv_class_cache[srv_type] = class_val

    return class_val


_message_class_cache = {}


def get_message_class(message_type):
    """
    Get the message class from a string representation.

    :param message_type: the type of message in the form `msg_pkg/Message`
    :type message_type: str

    :returns: None or the Class
    """
    if message_type in _message_class_cache:
        return _message_class_cache[message_type]

    logger = logging.get_logger('get_message_class')

    class_val = _get_rosidl_class_helper(message_type, MSG_MODE, logger)

    if class_val is not None:
        _message_class_cache[message_type] = class_val

    return class_val


_action_class_cache = {}


def get_action_class(action_type):
    """
    Get the action class from a string representation.

    :param action_type: the type of action in the form `action_pkg/Action`
    :type action_type: str

    :returns: None or the Class
    """
    if action_type in _action_class_cache:
        return _action_class_cache[action_type]

    logger = logging.get_logger('get_action_class')
    class_val = _get_rosidl_class_helper(action_type, ACTION_MODE, logger)

    if class_val is not None:
        _action_class_cache[action_type] = class_val

    return class_val


def get_message_text_from_class(msg_class):
    """Get a string representation of the message class."""
    msg_slot_dict = msg_class.get_fields_and_field_types()
    return ''.join(
        [f'{slot_type} {slot_name}\n' for
            slot_name, slot_type in msg_slot_dict.items()])


def get_service_text_from_class(srv_class):
    """Get a string representation of the message class."""
    srv_slot_dict_req = srv_class.Request.get_fields_and_field_types()
    srv_slot_dict_res = srv_class.Response.get_fields_and_field_types()
    srv_txt = [
        f'{slot_type} {slot_name}\n' for
        slot_name, slot_type in srv_slot_dict_req.items()]
    srv_txt.extend(['---\n'])
    srv_txt.extend([
        f'{slot_type} {slot_name}\n' for
        slot_name, slot_type in srv_slot_dict_res.items()])
    return ''.join(srv_txt)


def get_action_text_from_class(action_class):
    """Get a string representation of the message class."""
    action_slot_dict_goal = action_class.Goal.get_fields_and_field_types()
    action_slot_dict_res = action_class.Result.get_fields_and_field_types()
    action_slot_dict_feedback = action_class.Feedback.get_fields_and_field_types()
    action_txt = [
        f'{slot_type} {slot_name}\n' for
        slot_name, slot_type in action_slot_dict_goal.items()]
    action_txt.extend(['---\n'])
    action_txt.extend([
        f'{slot_type} {slot_name}\n' for
        slot_name, slot_type in action_slot_dict_res.items()])
    action_txt.extend(['---\n'])
    action_txt.extend([
        f'{slot_type} {slot_name}\n' for
        slot_name, slot_type in action_slot_dict_feedback.items()])
    return ''.join(action_txt)


def _get_constant_type_name(constant_type):
    """Get the IDL type name of a constant, such as ``int8`` or ``string``."""
    # Imported here so that rosidl_parser is only loaded when constants are requested
    from rosidl_parser.definition import AbstractWString
    from rosidl_parser.definition import BasicType

    if isinstance(constant_type, BasicType):
        return constant_type.typename
    # The parser gives string constants a bounded type whose maximum size is the
    # length of the value, so the bound is not part of the declared type
    if isinstance(constant_type, AbstractWString):
        return 'wstring'
    return 'string'


@functools.lru_cache(maxsize=None)
def _get_constant_types(msg_class):
    """
    Get the types of the constants of a message class from its IDL file.

    :param msg_class: The message class.
    :returns: A dictionary mapping each constant name to its IDL type name. The
        dictionary is empty if the IDL file of the interface cannot be found or parsed.
    """
    logger = logging.get_logger('get_message_constants_from_class')
    # For example, 'std_msgs.msg._header' and 'Header', or 'example_interfaces.srv._add_two_ints'
    # and 'AddTwoInts_Request', whose IDL file is 'srv/AddTwoInts.idl'.
    module_parts = msg_class.__module__.split('.')
    if len(module_parts) < 2:
        return {}
    package_name, subfolder = module_parts[0], module_parts[1]
    interface_name = msg_class.__name__.split('_')[0]
    try:
        from rosidl_parser.definition import Action
        from rosidl_parser.definition import IdlLocator
        from rosidl_parser.definition import Message
        from rosidl_parser.definition import Service
        from rosidl_parser.parser import parse_idl_file

        share_directory = get_package_share_directory(package_name)
        idl_path = os.path.join(subfolder, f'{interface_name}.idl')
        if not os.path.isfile(os.path.join(share_directory, idl_path)):
            return {}
        idl_file = parse_idl_file(IdlLocator(share_directory, idl_path))
    except Exception as e:  # noqa: B902
        logger.debug(f'Could not read the constant types of {msg_class.__name__}: {e}')
        return {}

    # Messages defined in the file, including the parts of services and actions
    messages = list(idl_file.content.get_elements_of_type(Message))
    for service in idl_file.content.get_elements_of_type(Service):
        messages += [service.request_message, service.response_message]
    for action in idl_file.content.get_elements_of_type(Action):
        messages += [action.goal, action.result, action.feedback]
    for message in messages:
        if message.structure.namespaced_type.name == msg_class.__name__:
            return {
                constant.name: _get_constant_type_name(constant.type)
                for constant in message.constants}
    return {}


def get_message_constants_from_class(msg_class):
    """
    Get the constants defined in a message.

    :param msg_class: The message class, or an instance of it.
    :returns: A list of ``(type, name, value)`` tuples, in the order in which the
        constants are defined. The type is the IDL type name, such as ``int8`` or
        ``string``, or an empty string if it cannot be determined.
    """
    if not isinstance(msg_class, type):
        msg_class = type(msg_class)
    # The generated metaclass stores the constants in a private dictionary
    metaclass = type(msg_class)
    constants = getattr(metaclass, f'_{metaclass.__name__.lstrip("_")}__constants', None)
    if not constants:
        return []
    types = _get_constant_types(msg_class)
    return [(types.get(name, ''), name, value) for name, value in constants.items()]
