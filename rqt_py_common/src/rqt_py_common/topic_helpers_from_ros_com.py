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

import sys

from operator import itemgetter

from rqt_py_common.message_helpers import get_message_class

class MsgSpecException(Exception):
    pass


class ROSTopicException(Exception):
    """
    Base exception class of rostopic-related errors
    """
    pass
class ROSTopicIOException(ROSTopicException):
    """
    rostopic errors related to network I/O failures
    """
    pass

def _get_nested_attribute(msg, nested_attributes):
    value = msg
    for attr in nested_attributes.split('/'):
        value = getattr(value, attr)
    return value

def _get_array_index_or_slice_object(index_string):
    # Taken from
    assert index_string != '', 'empty array index'
    index_string_parts = index_string.split(':')
    if len(index_string_parts) == 1:
        try:
            array_index = int(index_string_parts[0])
        except ValueError:
            assert False, "non-integer array index step '%s'" % index_string_parts[0]
        return array_index

    slice_args = [None, None, None]
    if index_string_parts[0] != '':
        try:
            slice_args[0] = int(index_string_parts[0])
        except ValueError:
            assert False, "non-integer slice start '%s'" % index_string_parts[0]
    if index_string_parts[1] != '':
        try:
            slice_args[1] = int(index_string_parts[1])
        except ValueError:
            assert False, "non-integer slice stop '%s'" % index_string_parts[1]
    if len(index_string_parts) > 2 and index_string_parts[2] != '':
            try:
                slice_args[2] = int(index_string_parts[2])
            except ValueError:
                assert False, "non-integer slice step '%s'" % index_string_parts[2]
    if len(index_string_parts) > 3:
        assert False, 'too many slice arguments'
    return slice(*slice_args)

def msgevalgen(pattern):
    """
    Generates a function that returns the relevant field(s) (aka 'subtopic(s)') of a Message object
    :param pattern: subtopic, e.g. /x[2:]/y[:-1]/z, ``str``
    :returns: function that converts a message into the desired value, ``fn(Message) -> value``
    """
    evals = []  # list of (field_name, slice_object) pairs
    fields = [f for f in pattern.split('/') if f]
    for f in fields:
        if '[' in f:
            field_name, rest = f.split('[', 1)
            if not rest.endswith(']'):
                print("missing closing ']' in slice spec '%s'" % f, file=sys.stderr)
                # return None
            rest = rest[:-1]  # slice content, removing closing bracket
            try:
                array_index_or_slice_object = _get_array_index_or_slice_object(rest)
            except AssertionError as e:
                print("field '%s' has invalid slice argument '%s': %s"
                      % (field_name, rest, str(e)), file=sys.stderr)
                # return None
            evals.append((field_name, array_index_or_slice_object))
        else:
            evals.append((f, None))

    def msgeval(msg, evals):
        for i, (field_name, slice_object) in enumerate(evals):
            try: # access field first
                msg = getattr(msg, field_name)
            except AttributeError:
                print("no field named %s in %s" % (field_name, pattern), file=sys.stderr)
                return None

            if slice_object is not None: # access slice
                try:
                    msg = msg.__getitem__(slice_object)
                except IndexError as e:
                    print("%s: %s" % (str(e), pattern), file=sys.stderr)
                    return None

                # if a list is returned here (i.e. not only a single element accessed),
                # we need to recursively call msg_eval() with the rest of evals
                # in order to handle nested slices
                if isinstance(msg, list):
                    rest = evals[i + 1:]
                    return [msgeval(m, rest) for m in msg]
        return msg

    return (lambda msg: msgeval(msg, evals)) if evals else None

# From ros_com/tools/rostopic.
def _get_topic_type_topic_name_and_msgevalgen(topic_names_and_types, path_to_field):
    """
    subroutine for get_topic_type_topic_name_and_msgevalgen
    (nearly identical to rostopic._get_topic_type,)

    See ``get_topic_type_topic_name_and_msgevalgen`` for full documentation

    :param topic_names_and_types:
    :type topic_names_and_types: list of tuples of form [(str, [str, str])] with
        the first item being the topic and the second item being a list of msgs
        being used on that topic

    :rtype: str, str, fn
    :raises: :exc:`ROSTopicException` If master cannot be contacted
    """
    # See if we can find a full match
    matches = []
    for (t_name, t_types) in topic_names_and_types:
        if t_name == path_to_field:
            for t_type in t_types:
                matches.append((t_name, t_type))

    if not matches:
        for (t_name, t_types) in topic_names_and_types:
            if path_to_field.startswith(t_name + '/'):
                for t_type in t_types:
                    matches.append((t_name, t_type))

        # choose longest match first
        matches.sort(key=itemgetter(0), reverse=True)

        # try to ignore messages which don't have the field specified as part of the topic name
        while matches:
            t_name, t_type = matches[0]
            msg_class = get_message_class(t_type)
            if not msg_class:
                # if any class is not fetchable skip ignoring any message types
                break

            msg = msg_class()
            nested_attributes = path_to_field[len(t_name) + 1:].rstrip('/')
            nested_attributes = nested_attributes.split('[')[0]
            if nested_attributes == '':
                break
            try:
                _get_nested_attribute(msg, nested_attributes)
            except AttributeError:
                # ignore this type since it does not have the requested field
                matches.pop(0)
                continue
            # Select this match
            matches = [(t_name, t_type)]
            break

    if matches:
        t_name, t_type = matches[0]
        # This is a relic from ros1 where rosgraph.names.ANYTYPE = '*'.
        # TODO: remove if this does nothing (which I suspect it does)
        if t_type == '*':
            return None, None, None
        return t_type, t_name, msgevalgen(path_to_field[len(t_name):])
    else:
        return None, None, None

def get_topic_type_topic_name_and_msgevalgen(node, path_to_field):
    """
    Get the topic type, the actual topic name.

    :param node: A ROS2 node
    :type node: ``rclpy.node.Node``

    :param path_to_field: path_to_field eg. /ns/node/topic/subfield_a[1]/subfeld_b/subfield_c
    :type path_to_field: ``str``

    :returns: topic type, real topic name and fn to evaluate the message instance
        if the topic points to a field within a topic, e.g. /rosout/msg. fn is None otherwise. ``(str, str, fn)``
        eg:
          in:   /ns/node/topic/subfield_a[1]/subfeld_b/subfield_c[1:]
          out:  ('/ns/node/topic', '/ns/node/topic', fn)

          in:   /ns/node/topic
          out:  ('/ns/node/topic', 'subfield_a[1]/subfeld_b/subfield_c', None)


    :raises: :exc:`ROSTopicException` If master cannot be contacted
    """
    topic_names_and_types = node.get_topic_names_and_types()
    topic_type, real_topic, msg_eval = \
        _get_topic_type_topic_name_and_msgevalgen(topic_names_and_types, path_to_field)

    if topic_type:
        return topic_type, real_topic, msg_eval

    return None, None, None