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
#  * Neither the name of PickNik Robotics nor the names of its
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


class TestMessageHelpers(unittest.TestCase):

    def test_get_message_class(self):
        from rqt_py_common.message_helpers import get_message_class
        # Check that we are able to import std_msgs/String
        from std_msgs.msg import String
        self.assertEqual(get_message_class('std_msgs/String'), String)
        # If no package is provided then we assume std_msgs
        self.assertEqual(get_message_class('String'), get_message_class('std_msgs/String'))
        self.assertEqual(get_message_class('string'), get_message_class('String'))
        # We test that we are able to import msgs from outside of std_msgs
        from rqt_py_common.msg import Val
        self.assertEqual(get_message_class('rqt_py_common/Val'), Val)

    def test_get_service_class(self):
        from rqt_py_common.message_helpers import get_service_class
        # Check that we are able to import std_msgs/String
        from rqt_py_common.srv import AddTwoInts
        # from std_srvs.srv import SetBool
        self.assertEqual(get_service_class('rqt_py_common/AddTwoInts'), AddTwoInts)
        # If no package is provided then we assume std_msgs
        self.assertEqual(get_service_class('Empty'), get_service_class('std_srvs/Empty'))
        self.assertEqual(get_service_class('empty'), get_service_class('Empty'))

    def test_get_message_text_from_class(self):
        from rqt_py_common.message_helpers import get_message_text_from_class
        from std_msgs.msg import MultiArrayDimension
        text = get_message_text_from_class(MultiArrayDimension)
        expected_text = 'string                        label\n' + \
                        'uint32                        size\n' + \
                        'uint32                        stride\n'
        self.assertEqual(text, expected_text)

    def test_get_service_text_from_class(self):
        from rqt_py_common.message_helpers import get_service_text_from_class
        from std_srvs.srv import SetBool
        text = get_service_text_from_class(SetBool)
        expected_text = \
            'bool                          data\n' + \
            '-----\n' + \
            'bool                          success\n' + \
            'string                        message\n'
        self.assertEqual(text, expected_text)
