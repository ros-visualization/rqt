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

import roslib
roslib.load_manifest('rqt_py_common')
import roslib.msgs
from rostopic import get_topic_type
from python_qt_binding.QtCore import qDebug 

def get_field_type(topic_name):
    # get topic_type and message_evaluator
    topic_type, real_topic_name, _ = get_topic_type(topic_name)
    if topic_type is None:
        #qDebug('topic_helpers.get_field_type(%s): get_topic_type failed' % (topic_name))
        return None, False

    message_class = roslib.message.get_message_class(topic_type)
    if message_class is None:
        qDebug('topic_helpers.get_field_type(%s): get_message_class(%s) failed' % (topic_name, topic_type))
        return None, False
    
    slot_path = topic_name[len(real_topic_name):]
    return get_slot_type(message_class, slot_path)

def get_slot_type(message_class, slot_path):
    is_array = False
    fields = [f for f in slot_path.split('/') if f]
    for field_name in fields:
        try:
            field_name, _, field_index = roslib.msgs.parse_type(field_name)
        except roslib.msgs.MsgSpecException:
            return None, False
        if field_name not in getattr(message_class, '__slots__', []):
            #qDebug('topic_helpers.get_slot_type(%s, %s): field not found: %s' % (message_class, slot_path, field_name))
            return None, False
        slot_type = message_class._slot_types[message_class.__slots__.index(field_name)]
        slot_type, slot_is_array, _ = roslib.msgs.parse_type(slot_type)
        is_array = slot_is_array and field_index is None
        if roslib.msgs.is_valid_constant_type(slot_type):
            if slot_type == 'string':
                message_class = str
            elif slot_type == 'bool':
                message_class = bool
            else:
                message_class = type(roslib.msgs._convert_val(slot_type, 0))
        else:
            message_class = roslib.message.get_message_class(slot_type)
    return message_class, is_array

def is_slot_numeric(topic_name):
    field_type, is_array = get_field_type(topic_name)
    if field_type in (int, float):
        if is_array:
            message = 'topic "%s" is numeric array: %s[]' % (topic_name, field_type)
        else:
            message = 'topic "%s" is numeric: %s' % (topic_name, field_type)
        return True, is_array, message

    return False, is_array, 'topic "%s" is NOT numeric: %s' % (topic_name, field_type)
