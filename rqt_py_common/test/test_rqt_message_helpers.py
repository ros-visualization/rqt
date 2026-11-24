#!/usr/bin/env python3

# Software License Agreement (BSD License)
#
# Copyright (c) 2018, PickNik Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Author: Michael Lautman

import unittest


class TestMessageHelpers(unittest.TestCase):  # noqa: D101

    def test_get_message_class(self):  # noqa: D102
        from rqt_py_common.message_helpers import get_message_class
        from rqt_py_common_test_interfaces.msg import Val
        self.assertEqual(get_message_class('rqt_py_common_test_interfaces/Val'), Val)

    def test_get_service_class(self):  # noqa: D102
        from rqt_py_common.message_helpers import get_service_class
        from rqt_py_common_test_interfaces.srv import AddTwoInts
        self.assertEqual(get_service_class('rqt_py_common_test_interfaces/AddTwoInts'), AddTwoInts)

    def test_get_message_text_from_class(self):  # noqa: D102
        from rqt_py_common.message_helpers import get_message_text_from_class
        from rqt_py_common_test_interfaces.msg import ArrayVal, Val
        text = get_message_text_from_class(ArrayVal)
        expected_text = 'rqt_py_common_test_interfaces/Val[5] vals\n'
        self.assertEqual(text, expected_text)

        text = get_message_text_from_class(Val)
        expected_text1 = 'float64[5] floats\n'  # .msg based type name
        expected_text2 = 'double[5] floats\n'  # .idl based type name
        self.assertTrue(text in (expected_text1, expected_text2))

    def test_get_service_text_from_class(self):  # noqa: D102
        from rqt_py_common.message_helpers import get_service_text_from_class
        from rqt_py_common_test_interfaces.srv import AddTwoInts
        text = get_service_text_from_class(AddTwoInts)
        expected_text = \
            'int64 a\n' + \
            'int64 b\n' + \
            '---\n' + \
            'int64 sum\n'
        self.assertEqual(text, expected_text)

    def test_get_message_constants_from_class(self):  # noqa: D102
        from rqt_py_common.message_helpers import get_message_constants_from_class
        from rqt_py_common_test_interfaces.msg import Constants, Val
        # The class and an instance of it give the same constants, in definition order
        for msg in (Constants, Constants()):
            constants = get_message_constants_from_class(msg)
            self.assertEqual(
                [(name, value) for _, name, value in constants],
                [('INT8_CONST', -5), ('STRING_CONST', 'hello')])
        # A message without constants
        self.assertEqual(get_message_constants_from_class(Val), [])

    def test_get_message_constants_types_from_idl(self):  # noqa: D102
        import os
        from unittest import mock

        from rqt_py_common import message_helpers
        import rqt_py_common_test_interfaces
        from rqt_py_common_test_interfaces.msg import Constants
        # The test interfaces are not installed, so the share directory is replaced by the
        # directory where rosidl_adapter generates their IDL files in the build tree
        build_directory = os.path.dirname(os.path.dirname(os.path.dirname(
            os.path.abspath(rqt_py_common_test_interfaces.__file__))))
        idl_directory = os.path.join(
            build_directory, 'rosidl_adapter', 'rqt_py_common_test_interfaces')
        self.assertTrue(os.path.isfile(os.path.join(idl_directory, 'msg', 'Constants.idl')))
        message_helpers._get_constant_types.cache_clear()
        try:
            with mock.patch.object(
                    message_helpers, 'get_package_share_directory', return_value=idl_directory):
                constants = message_helpers.get_message_constants_from_class(Constants)
        finally:
            message_helpers._get_constant_types.cache_clear()
        self.assertEqual(
            constants,
            [('int8', 'INT8_CONST', -5), ('string', 'STRING_CONST', 'hello')])

    def test_get_constant_type_name(self):  # noqa: D102
        from rosidl_parser.definition import BasicType
        from rosidl_parser.definition import BoundedString
        from rosidl_parser.definition import BoundedWString
        from rosidl_parser.definition import UnboundedString
        from rosidl_parser.definition import UnboundedWString
        from rqt_py_common.message_helpers import _get_constant_type_name
        self.assertEqual(_get_constant_type_name(BasicType('int8')), 'int8')
        self.assertEqual(_get_constant_type_name(UnboundedString()), 'string')
        self.assertEqual(_get_constant_type_name(UnboundedWString()), 'wstring')
        # The parser bounds string constants by the length of their value
        self.assertEqual(_get_constant_type_name(BoundedString(12)), 'string')
        self.assertEqual(_get_constant_type_name(BoundedWString(5)), 'wstring')

    def test_get_all_message_types(self):  # noqa: D102
        from rqt_py_common.message_helpers import get_all_message_types
        all_msgs = get_all_message_types()
        self.assertTrue('rqt_py_common_test_interfaces' in all_msgs.keys())
        self.assertTrue('ArrayVal' in all_msgs['rqt_py_common_test_interfaces'])

    def test_get_all_service_types(self):  # noqa: D102
        from rqt_py_common.message_helpers import get_all_service_types
        all_srvs = get_all_service_types()
        self.assertTrue('rqt_py_common_test_interfaces' in all_srvs.keys())
        self.assertTrue('AddTwoInts' in all_srvs['rqt_py_common_test_interfaces'])
