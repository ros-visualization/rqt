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
from rostopic import get_topic_type

def get_field_type(topic_name):
    # get message
    topic_type, _, message_evaluator = get_topic_type(topic_name)
    if topic_type is None:
        return None
    message_class = roslib.message.get_message_class(topic_type)
    if message_class is None:
        return None
    message = message_class()

    # return field type
    if message_evaluator:
        try:
            field_type = type(message_evaluator(message))
        except Exception:
            field_type = None
    else:
        field_type = type(message)

    return field_type

def is_slot_numeric(topic_name):
    # check for numeric field type
    topic_base = topic_name.rsplit('[', 1)[0]
    field_type = get_field_type(topic_base)
    if field_type in (int, float):
        return True, 'topic "%s" is numeric: %s' % (topic_name, field_type)
    elif field_type in (tuple, list) and topic_name.endswith(']'):
        return True, 'topic "%s" is a list, hoping for a valid index...' % (topic_name)

    return False, 'topic "%s" is NOT numeric: %s' % (topic_name, field_type)
