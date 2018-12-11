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


class TestMessageHelpers(unittest.TestCase):  # noqa: D101

    def test_get_message_class(self):  # noqa: D102
        from rqt_py_common.message_helpers import get_message_class
        from rqt_py_common.msg import Val
        self.assertEqual(get_message_class('rqt_py_common/Val'), Val)

    def test_get_service_class(self):  # noqa: D102
        from rqt_py_common.message_helpers import get_service_class
        from rqt_py_common.srv import AddTwoInts
        self.assertEqual(get_service_class('rqt_py_common/AddTwoInts'), AddTwoInts)

    def test_get_message_text_from_class(self):  # noqa: D102
        from rqt_py_common.message_helpers import get_message_text_from_class
        from rqt_py_common.msg import ArrayVal, Val
        text = get_message_text_from_class(ArrayVal)
        expected_text = 'rqt_py_common/Val[5] vals\n'
        self.assertEqual(text, expected_text)

        text = get_message_text_from_class(Val)
        expected_text = 'float64[5] floats\n'
        self.assertEqual(text, expected_text)

    def test_get_service_text_from_class(self):  # noqa: D102
        from rqt_py_common.message_helpers import get_service_text_from_class
        from rqt_py_common.srv import AddTwoInts
        text = get_service_text_from_class(AddTwoInts)
        expected_text = \
            'int64 a\n' + \
            'int64 b\n' + \
            '---\n' + \
            'int64 sum\n'
        self.assertEqual(text, expected_text)

    def test_get_all_message_types(self):  # noqa: D102
        from rqt_py_common.message_helpers import get_all_message_types
        all_msgs = get_all_message_types()
        self.assertTrue('rqt_py_common' in all_msgs.keys())
        self.assertTrue('ArrayVal' in all_msgs['rqt_py_common'])

    def test_get_all_service_types(self):  # noqa: D102
        from rqt_py_common.message_helpers import get_all_service_types
        all_srvs = get_all_service_types()
        self.assertTrue('rqt_py_common' in all_srvs.keys())
        self.assertTrue('AddTwoInts' in all_srvs['rqt_py_common'])
