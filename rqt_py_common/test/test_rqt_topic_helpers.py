#!/usr/bin/env python3

# Software License Agreement (BSD License)
#
# Copyright (c) 2018, PickNik Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Author: Michael Lautman

import unittest


class TestTopicHelpers(unittest.TestCase): # noqa

    def test_get_message_class(self):  # noqa
        from rqt_py_common.topic_helpers import get_message_class
        # Check that we are able to import std_msgs/String
        from std_msgs.msg import String
        self.assertEqual(get_message_class('std_msgs/String'), String)
        # If no package is provided then we assume std_msgs
        self.assertEqual(get_message_class('String'), get_message_class('std_msgs/String'))
        self.assertEqual(get_message_class('string'), get_message_class('String'))
        # We test that we are able to import msgs from outside of std_msgs
        from rqt_py_common.msg import Val
        self.assertEqual(get_message_class('rqt_py_common/Val'), Val)

    def test_get_slot_type(self):  # noqa
        from rqt_py_common.topic_helpers import get_slot_type
        from rqt_py_common.topic_helpers import get_message_class
        from rqt_py_common.msg import ArrayVal
        # Check that we are able to import std_msgs/String
        path = '_vals/_floats'
        message_class = ArrayVal
        message_type, is_array = get_slot_type(message_class, path)
        self.assertTrue(is_array)
        self.assertEqual(message_type, float)

        path = '/_vals'
        message_class = ArrayVal
        message_type, is_array = get_slot_type(message_class, path)
        self.assertTrue(is_array)
        self.assertEqual(message_type, get_message_class('rqt_py_common/Val'))


    def test_get_field_type(self):  # noqa
        from rqt_py_common.topic_helpers import _get_field_type
        from rqt_py_common.msg import ArrayVal, Val
        target = '/example_topic/_vals/_floats'
        topic_names_and_types = [('/example_topic', ['rqt_py_common/ArrayVal'])]
        target_class, is_array = _get_field_type(topic_names_and_types, target)
        self.assertTrue(is_array)
        self.assertEqual(target_class, float)

        target = '/example_topic'
        target_class, is_array = _get_field_type(topic_names_and_types, target)
        self.assertFalse(is_array)
        self.assertEqual(target_class, ArrayVal)

        target = '/example_topic/_vals'
        target_class, is_array = _get_field_type(topic_names_and_types, target)
        self.assertTrue(is_array)
        self.assertEqual(target_class, Val)
