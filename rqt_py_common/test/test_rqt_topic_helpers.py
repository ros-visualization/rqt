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


class TestTopicHelpers(unittest.TestCase):  # noqa: D101
    example_topic_names_and_types = [
        ('/example', ['rqt_py_common/Val']),
        ('/example/vals', ['rqt_py_common/Val']),
        ('/example_topic', ['rqt_py_common/Val']),
        ('/example/topic', ['rqt_py_common/ArrayVal']),
    ]

    def test_get_slot_type(self):  # noqa: D102
        from rqt_py_common.topic_helpers import get_slot_type
        from rqt_py_common.message_helpers import get_message_class
        from rqt_py_common.msg import ArrayVal
        path = 'vals/floats'
        message_class = ArrayVal
        message_type, is_array = get_slot_type(message_class, path)
        self.assertTrue(is_array)
        self.assertEqual(message_type, float)

        path = '/vals'
        message_class = ArrayVal
        message_type, is_array = get_slot_type(message_class, path)
        self.assertTrue(is_array)
        self.assertEqual(message_type, get_message_class('rqt_py_common/Val'))

    def test_get_field_type(self):  # noqa: D102
        from rqt_py_common.topic_helpers import _get_field_type
        from rqt_py_common.msg import ArrayVal, Val

        val_slots = ["", "floats", "unbounded_floats", "bounded_floats", "single_float"]
        val_slot_is_arr = [False, True, True, True, False]
        val_slot_class = [Val, float, float, float, float]

        array_val_slots = ["", "vals", "unbounded_vals", "bounded_vals", "single_val"]
        array_val_slot_is_arr = [False, True, True, True, False]
        array_val_slot_class = [ArrayVal, Val, Val, Val, Val]

        array_val_slots_len = len(array_val_slots)
        for i in [i for i in range(array_val_slots_len) if array_val_slots[i]]:
            for j, val_value in [(j, v) for (j, v) in enumerate(val_slots) if v]:
                if val_value:
                    array_val_slots.append(array_val_slots[i] + "/" + val_value)
                    array_val_slot_is_arr.append(val_slot_is_arr[j])
                    array_val_slot_class.append(val_slot_class[j])

        for topic, msg_type in TestTopicHelpers.example_topic_names_and_types:
            top_level_class = None
            slot_values = []
            slot_is_arr = []
            slot_class = []
            if msg_type[0] == 'rqt_py_common/Val':
                top_level_class = Val
                slot_values = val_slots
                slot_is_arr = val_slot_is_arr
                slot_class = val_slot_class
            elif msg_type[0] == 'rqt_py_common/ArrayVal':
                top_level_class = ArrayVal
                slot_values = array_val_slots
                slot_is_arr = array_val_slot_is_arr
                slot_class = array_val_slot_class

            for i, slot_value in enumerate(slot_values):
                target = topic + "/" + slot_value
                target_class, is_array = _get_field_type(
                    TestTopicHelpers.example_topic_names_and_types, target
                )

                if slot_is_arr[i]:
                    self.assertTrue(is_array)
                else:
                    self.assertFalse(is_array)
                self.assertEqual(target_class, slot_class[i])

    def test_get_slot_class_and_field_information(self):
        from rqt_py_common.topic_helpers import get_slot_class_and_field_information
        from rqt_py_common.msg import ArrayVal, Val

        val_slots = ["", "floats", "unbounded_floats", "bounded_floats", "single_float"]
        val_slot_class = [Val, float, float, float, float]
        val_slot_array_info = [
            {   # field = None   and field_type rqt_py_common/Val
                'base_type_string': 'rqt_py_common/Val', 'is_array': False,
                'is_static_array': False, 'static_array_size': -1,
                'is_bounded_array': False, 'bounded_array_size': -1,
                'is_unbounded_array': False,
                'is_bounded_string': False, 'bounded_string_size': -1,
            },{ # field = "floats" and field_type = double[5]
                'base_type_string': 'double', 'is_array': True,
                'is_static_array': True, 'static_array_size': 5,
                'is_bounded_array': False, 'bounded_array_size': -1,
                'is_unbounded_array': False,
                'is_bounded_string': False, 'bounded_string_size': -1
            },{ # field = "unbounded_floats" and field_type = sequence<double>
                'base_type_string': 'double', 'is_array': True,
                'is_static_array': False, 'static_array_size': -1,
                'is_bounded_array': False, 'bounded_array_size': -1,
                'is_unbounded_array': True,
                'is_bounded_string': False, 'bounded_string_size': -1
            },{ # field = "bounded_floats" and field_type = sequence<double, 3>
                'base_type_string': 'double', 'is_array': True,
                'is_static_array': False, 'static_array_size': -1,
                'is_bounded_array': True, 'bounded_array_size': 3,
                'is_unbounded_array': False,
                'is_bounded_string': False, 'bounded_string_size': -1
            },{ # field = "single_float" and field_type = double
                'base_type_string': 'double', 'is_array': False,
                'is_static_array': False, 'static_array_size': -1,
                'is_bounded_array': False, 'bounded_array_size': -1,
                'is_unbounded_array': False,
                'is_bounded_string': False, 'bounded_string_size': -1
            }]

        array_val_slots = ["", "vals", "unbounded_vals", "bounded_vals", "single_val"]
        array_val_slot_array_info = [
            {   # Field Name "" and field_type = rqt_py_common/ArrayVal
                'base_type_string': 'rqt_py_common/ArrayVal', 'is_array': False,
                'is_static_array': False, 'static_array_size': -1,
                'is_bounded_array': False, 'bounded_array_size': -1,
                'is_unbounded_array': False,
                'is_bounded_string': False, 'bounded_string_size': -1,
            },{ # Field Name "vals" and field_type = rqt_py_common/Val[5]
                'base_type_string': 'rqt_py_common/Val', 'is_array': True,
                'is_static_array': True, 'static_array_size': 5,
                'is_bounded_array': False, 'bounded_array_size': -1,
                'is_unbounded_array': False,
                'is_bounded_string': False, 'bounded_string_size': -1
            },{ # Field Name "unbounded_vals" and field_type = sequence<rqt_py_common/Val>
                'base_type_string': 'rqt_py_common/Val', 'is_array': True,
                'is_static_array': False, 'static_array_size': -1,
                'is_bounded_array': False, 'bounded_array_size': -1,
                'is_unbounded_array': True,
                'is_bounded_string': False, 'bounded_string_size': -1
            },{ # Field Name "bounded_vals" and field_type = sequence<rqt_py_common/Val, 5>
                'base_type_string': 'rqt_py_common/Val', 'is_array': True,
                'is_static_array': False, 'static_array_size': -1,
                'is_bounded_array': True, 'bounded_array_size': 5,
                'is_unbounded_array': False,
                'is_bounded_string': False, 'bounded_string_size': -1
            },{ # Field Name "single_val" and field_type = rqt_py_common/Val
                'base_type_string': 'rqt_py_common/Val', 'is_array': False,
                'is_static_array': False, 'static_array_size': -1,
                'is_bounded_array': False, 'bounded_array_size': -1,
                'is_unbounded_array': False,
                'is_bounded_string': False, 'bounded_string_size': -1
                }]
        array_val_slot_class = [ArrayVal, Val, Val, Val, Val]

        array_val_slots_len = len(array_val_slots)
        for i in [i for i in range(array_val_slots_len) if array_val_slots[i]]:
            for j, val_value in [(j, v) for (j, v) in enumerate(val_slots) if v]:
                if val_value:
                    array_val_slots.append(array_val_slots[i] + "/" + val_value)
                    array_val_slot_array_info.append(val_slot_array_info[j])
                    array_val_slot_class.append(val_slot_class[j])

        for msg_type in ['rqt_py_common/Val', 'rqt_py_common/ArrayVal']:
            top_level_class = None
            slot_values = []
            slot_array_info = []
            slot_class = []
            if msg_type == 'rqt_py_common/Val':
                top_level_class = Val
                slot_values = val_slots
                slot_array_info = val_slot_array_info
                slot_class = val_slot_class
            elif msg_type == 'rqt_py_common/ArrayVal':
                top_level_class = ArrayVal
                slot_values = array_val_slots
                slot_array_info = array_val_slot_array_info
                slot_class = array_val_slot_class

            for i, slot_value in enumerate(slot_values):
                target = "/" + slot_value
                target_class, field_info = get_slot_class_and_field_information(
                    top_level_class, slot_value
                )

                self.assertEqual(target_class, slot_class[i])
                for info_k, info_v in slot_array_info[i].items():
                    self.assertTrue(info_k in field_info)
                    self.assertEqual(info_v, field_info[info_k])

    def test_get_field_type_array_information(self):
        from rqt_py_common.topic_helpers import get_field_type_array_information
        slot_type_to_info = {
            'boolean' : {
                "base_type_string": "boolean", "is_array": False,
                "is_static_array": False, "static_array_size": -1,
                "is_bounded_array": False, "bounded_array_size": -1,
                "is_unbounded_array": False,
                "is_bounded_string": False, "bounded_string_size": -1},
            'octet' : {
                "base_type_string": "octet", "is_array": False,
                "is_static_array": False, "static_array_size": -1,
                "is_bounded_array": False, "bounded_array_size": -1,
                "is_unbounded_array": False,
                "is_bounded_string": False, "bounded_string_size": -1},
            'float' : {
                "base_type_string": "float", "is_array": False,
                "is_static_array": False, "static_array_size": -1,
                "is_bounded_array": False, "bounded_array_size": -1,
                "is_unbounded_array": False,
                "is_bounded_string": False, "bounded_string_size": -1},
            'double' : {
                "base_type_string": "double", "is_array": False,
                "is_static_array": False, "static_array_size": -1,
                "is_bounded_array": False, "bounded_array_size": -1,
                "is_unbounded_array": False,
                "is_bounded_string": False, "bounded_string_size": -1},
            'uint8' : {
                "base_type_string": "uint8", "is_array": False,
                "is_static_array": False, "static_array_size": -1,
                "is_bounded_array": False, "bounded_array_size": -1,
                "is_unbounded_array": False,
                "is_bounded_string": False, "bounded_string_size": -1},
            'int8' : {
                "base_type_string": "int8", "is_array": False,
                "is_static_array": False, "static_array_size": -1,
                "is_bounded_array": False, "bounded_array_size": -1,
                "is_unbounded_array": False,
                "is_bounded_string": False, "bounded_string_size": -1},
            'int16' : {
                "base_type_string": "int16", "is_array": False,
                "is_static_array": False, "static_array_size": -1,
                "is_bounded_array": False, "bounded_array_size": -1,
                "is_unbounded_array": False,
                "is_bounded_string": False, "bounded_string_size": -1},
            'int32' : {
                "base_type_string": "int32", "is_array": False,
                "is_static_array": False, "static_array_size": -1,
                "is_bounded_array": False, "bounded_array_size": -1,
                "is_unbounded_array": False,
                "is_bounded_string": False, "bounded_string_size": -1},
            'int64' : {
                "base_type_string": "int64", "is_array": False,
                "is_static_array": False, "static_array_size": -1,
                "is_bounded_array": False, "bounded_array_size": -1,
                "is_unbounded_array": False,
                "is_bounded_string": False, "bounded_string_size": -1},
            'uint8' : {
                "base_type_string": "uint8", "is_array": False,
                "is_static_array": False, "static_array_size": -1,
                "is_bounded_array": False, "bounded_array_size": -1,
                "is_unbounded_array": False,
                "is_bounded_string": False, "bounded_string_size": -1},
            'uint16' : {
                "base_type_string": "uint16", "is_array": False,
                "is_static_array": False, "static_array_size": -1,
                "is_bounded_array": False, "bounded_array_size": -1,
                "is_unbounded_array": False,
                "is_bounded_string": False, "bounded_string_size": -1},
            'uint32' : {
                "base_type_string": "uint32", "is_array": False,
                "is_static_array": False, "static_array_size": -1,
                "is_bounded_array": False, "bounded_array_size": -1,
                "is_unbounded_array": False,
                "is_bounded_string": False, "bounded_string_size": -1},
            'uint64' : {
                "base_type_string": "uint64", "is_array": False,
                "is_static_array": False, "static_array_size": -1,
                "is_bounded_array": False, "bounded_array_size": -1,
                "is_unbounded_array": False,
                "is_bounded_string": False, "bounded_string_size": -1},
            'string' : {
                "base_type_string": "string", "is_array": False,
                "is_static_array": False, "static_array_size": -1,
                "is_bounded_array": False, "bounded_array_size": -1,
                "is_unbounded_array": False,
                "is_bounded_string": False, "bounded_string_size": -1},
            'int8[5]' : {
                "base_type_string": "int8", "is_array": True,
                "is_static_array": True, "static_array_size": 5,
                "is_bounded_array": False, "bounded_array_size": -1,
                "is_unbounded_array": False,
                "is_bounded_string": False, "bounded_string_size": -1},
            'my_msgs/Wstring' : {
                "base_type_string": "my_msgs/Wstring", "is_array": False,
                "is_static_array": False, "static_array_size": -1,
                "is_bounded_array": False, "bounded_array_size": -1,
                "is_unbounded_array": False,
                "is_bounded_string": False, "bounded_string_size": -1},
            'my_msgs/Wstring[1]' : {
                "base_type_string": "my_msgs/Wstring", "is_array": True,
                "is_static_array": True, "static_array_size": 1,
                "is_bounded_array": False, "bounded_array_size": -1,
                "is_unbounded_array": False,
                "is_bounded_string": False, "bounded_string_size": -1},
            'sequence<my_msgs/Wstring>' : {
                "base_type_string": "my_msgs/Wstring", "is_array": True,
                "is_static_array": False, "static_array_size": -1,
                "is_bounded_array": False, "bounded_array_size": -1,
                "is_unbounded_array": True,
                "is_bounded_string": False, "bounded_string_size": -1},
            'sequence<my_msgs/Wstring, 10>' : {
                "base_type_string": "my_msgs/Wstring", "is_array": True,
                "is_static_array": False, "static_array_size": -1,
                "is_bounded_array": True, "bounded_array_size": 10,
                "is_unbounded_array": False,
                "is_bounded_string": False, "bounded_string_size": -1},
            'sequence<my_msgs/Wstring, 10>' : {
                "base_type_string": "my_msgs/Wstring", "is_array": True,
                "is_static_array": False, "static_array_size": -1,
                "is_bounded_array": True, "bounded_array_size": 10,
                "is_unbounded_array": False,
                "is_bounded_string": False, "bounded_string_size": -1},
            'sequence<my_msgs/Wstring>' : {
                "base_type_string": "my_msgs/Wstring", "is_array": True,
                "is_static_array": False, "static_array_size": -1,
                "is_bounded_array": False, "bounded_array_size": -1,
                "is_unbounded_array": True,
                "is_bounded_string": False, "bounded_string_size": -1},
            'sequence<int8>' : {
                "base_type_string": "int8", "is_array": True,
                "is_static_array": False, "static_array_size": -1,
                "is_bounded_array": False, "bounded_array_size": -1,
                "is_unbounded_array": True,
                "is_bounded_string": False, "bounded_string_size": -1},
            'sequence<int8, 9>' : {
                "base_type_string": "int8", "is_array": True,
                "is_static_array": False, "static_array_size": -1,
                "is_bounded_array": True, "bounded_array_size": 9,
                "is_unbounded_array": False,
                "is_bounded_string": False, "bounded_string_size": -1},
            'string[100]' : {
                "base_type_string": "string", "is_array": True,
                "is_static_array": True, "static_array_size": 100,
                "is_bounded_array": False, "bounded_array_size": -1,
                "is_unbounded_array": False,
                "is_bounded_string": False, "bounded_string_size": -1},
            'sequence<string>' : {
                "base_type_string": "string", "is_array": True,
                "is_static_array": False, "static_array_size": -1,
                "is_bounded_array": False, "bounded_array_size": -1,
                "is_unbounded_array": True,
                "is_bounded_string": False, "bounded_string_size": -1},
            'string<999>[34]' : {
                "base_type_string": "string", "is_array": True,
                "is_static_array": True, "static_array_size": 34,
                "is_bounded_array": False, "bounded_array_size": -1,
                "is_unbounded_array": False,
                "is_bounded_string": True, "bounded_string_size": 999},
            'sequence<string<1>, 10>' : {
                "base_type_string": "string", "is_array": True,
                "is_static_array": False, "static_array_size": -1,
                "is_bounded_array": True, "bounded_array_size": 10,
                "is_unbounded_array": False,
                "is_bounded_string": True, "bounded_string_size": 1},
            'sequence<string<40>>' : {
                "base_type_string": "string", "is_array": True,
                "is_static_array": False, "static_array_size": -1,
                "is_bounded_array": False, "bounded_array_size": -1,
                "is_unbounded_array": True,
                "is_bounded_string": True, "bounded_string_size": 40},
            'sequence<string, 200>' : {
                "base_type_string": "string", "is_array": True,
                "is_static_array": False, "static_array_size": -1,
                "is_bounded_array": True, "bounded_array_size": 200,
                "is_unbounded_array": False,
                "is_bounded_string": False, "bounded_string_size": -1},
        }
        for slot, array_info in slot_type_to_info.items():
            generated_array_info = get_field_type_array_information(slot)
            for info_k, info_v in array_info.items():
                self.assertTrue(info_k in generated_array_info)
                self.assertEqual(info_v, generated_array_info[info_k])

    def test_autogenerated_get_field_type(self):
        '''
        Test get field type for all of the example_topic_names_and_types

        These are generated by generate_test_get_field_type (below). In case the above
        test fails these are useful because they have more useful test failure
        information
        '''
        from rqt_py_common.topic_helpers import _get_field_type
        from rqt_py_common.msg import ArrayVal, Val

        target = "/example/"
        target_class, is_array = _get_field_type(self.example_topic_names_and_types, target)
        self.assertFalse(is_array)
        self.assertEqual(target_class, Val)

        target = "/example/floats"
        target_class, is_array = _get_field_type(self.example_topic_names_and_types, target)
        self.assertTrue(is_array)
        self.assertEqual(target_class, float)

        target = "/example/unbounded_floats"
        target_class, is_array = _get_field_type(self.example_topic_names_and_types, target)
        self.assertTrue(is_array)
        self.assertEqual(target_class, float)

        target = "/example/bounded_floats"
        target_class, is_array = _get_field_type(self.example_topic_names_and_types, target)
        self.assertTrue(is_array)
        self.assertEqual(target_class, float)

        target = "/example/single_float"
        target_class, is_array = _get_field_type(self.example_topic_names_and_types, target)
        self.assertFalse(is_array)
        self.assertEqual(target_class, float)

        target = "/example/vals/"
        target_class, is_array = _get_field_type(self.example_topic_names_and_types, target)
        self.assertFalse(is_array)
        self.assertEqual(target_class, Val)

        target = "/example/vals/floats"
        target_class, is_array = _get_field_type(self.example_topic_names_and_types, target)
        self.assertTrue(is_array)
        self.assertEqual(target_class, float)

        target = "/example/vals/unbounded_floats"
        target_class, is_array = _get_field_type(self.example_topic_names_and_types, target)
        self.assertTrue(is_array)
        self.assertEqual(target_class, float)

        target = "/example/vals/bounded_floats"
        target_class, is_array = _get_field_type(self.example_topic_names_and_types, target)
        self.assertTrue(is_array)
        self.assertEqual(target_class, float)

        target = "/example/vals/single_float"
        target_class, is_array = _get_field_type(self.example_topic_names_and_types, target)
        self.assertFalse(is_array)
        self.assertEqual(target_class, float)

        target = "/example_topic/"
        target_class, is_array = _get_field_type(self.example_topic_names_and_types, target)
        self.assertFalse(is_array)
        self.assertEqual(target_class, Val)

        target = "/example_topic/floats"
        target_class, is_array = _get_field_type(self.example_topic_names_and_types, target)
        self.assertTrue(is_array)
        self.assertEqual(target_class, float)

        target = "/example_topic/unbounded_floats"
        target_class, is_array = _get_field_type(self.example_topic_names_and_types, target)
        self.assertTrue(is_array)
        self.assertEqual(target_class, float)

        target = "/example_topic/bounded_floats"
        target_class, is_array = _get_field_type(self.example_topic_names_and_types, target)
        self.assertTrue(is_array)
        self.assertEqual(target_class, float)

        target = "/example_topic/single_float"
        target_class, is_array = _get_field_type(self.example_topic_names_and_types, target)
        self.assertFalse(is_array)
        self.assertEqual(target_class, float)

        target = "/example/topic/"
        target_class, is_array = _get_field_type(self.example_topic_names_and_types, target)
        self.assertFalse(is_array)
        self.assertEqual(target_class, ArrayVal)

        target = "/example/topic/vals"
        target_class, is_array = _get_field_type(self.example_topic_names_and_types, target)
        self.assertTrue(is_array)
        self.assertEqual(target_class, Val)

        target = "/example/topic/unbounded_vals"
        target_class, is_array = _get_field_type(self.example_topic_names_and_types, target)
        self.assertTrue(is_array)
        self.assertEqual(target_class, Val)

        target = "/example/topic/bounded_vals"
        target_class, is_array = _get_field_type(self.example_topic_names_and_types, target)
        self.assertTrue(is_array)
        self.assertEqual(target_class, Val)

        target = "/example/topic/single_val"
        target_class, is_array = _get_field_type(self.example_topic_names_and_types, target)
        self.assertFalse(is_array)
        self.assertEqual(target_class, Val)

        target = "/example/topic/vals/floats"
        target_class, is_array = _get_field_type(self.example_topic_names_and_types, target)
        self.assertTrue(is_array)
        self.assertEqual(target_class, float)

        target = "/example/topic/vals/unbounded_floats"
        target_class, is_array = _get_field_type(self.example_topic_names_and_types, target)
        self.assertTrue(is_array)
        self.assertEqual(target_class, float)

        target = "/example/topic/vals/bounded_floats"
        target_class, is_array = _get_field_type(self.example_topic_names_and_types, target)
        self.assertTrue(is_array)
        self.assertEqual(target_class, float)

        target = "/example/topic/vals/single_float"
        target_class, is_array = _get_field_type(self.example_topic_names_and_types, target)
        self.assertFalse(is_array)
        self.assertEqual(target_class, float)

        target = "/example/topic/unbounded_vals/floats"
        target_class, is_array = _get_field_type(self.example_topic_names_and_types, target)
        self.assertTrue(is_array)
        self.assertEqual(target_class, float)

        target = "/example/topic/unbounded_vals/unbounded_floats"
        target_class, is_array = _get_field_type(self.example_topic_names_and_types, target)
        self.assertTrue(is_array)
        self.assertEqual(target_class, float)

        target = "/example/topic/unbounded_vals/bounded_floats"
        target_class, is_array = _get_field_type(self.example_topic_names_and_types, target)
        self.assertTrue(is_array)
        self.assertEqual(target_class, float)

        target = "/example/topic/unbounded_vals/single_float"
        target_class, is_array = _get_field_type(self.example_topic_names_and_types, target)
        self.assertFalse(is_array)
        self.assertEqual(target_class, float)

        target = "/example/topic/bounded_vals/floats"
        target_class, is_array = _get_field_type(self.example_topic_names_and_types, target)
        self.assertTrue(is_array)
        self.assertEqual(target_class, float)

        target = "/example/topic/bounded_vals/unbounded_floats"
        target_class, is_array = _get_field_type(self.example_topic_names_and_types, target)
        self.assertTrue(is_array)
        self.assertEqual(target_class, float)

        target = "/example/topic/bounded_vals/bounded_floats"
        target_class, is_array = _get_field_type(self.example_topic_names_and_types, target)
        self.assertTrue(is_array)
        self.assertEqual(target_class, float)

        target = "/example/topic/bounded_vals/single_float"
        target_class, is_array = _get_field_type(self.example_topic_names_and_types, target)
        self.assertFalse(is_array)
        self.assertEqual(target_class, float)

        target = "/example/topic/single_val/floats"
        target_class, is_array = _get_field_type(self.example_topic_names_and_types, target)
        self.assertTrue(is_array)
        self.assertEqual(target_class, float)

        target = "/example/topic/single_val/unbounded_floats"
        target_class, is_array = _get_field_type(self.example_topic_names_and_types, target)
        self.assertTrue(is_array)
        self.assertEqual(target_class, float)

        target = "/example/topic/single_val/bounded_floats"
        target_class, is_array = _get_field_type(self.example_topic_names_and_types, target)
        self.assertTrue(is_array)
        self.assertEqual(target_class, float)

        target = "/example/topic/single_val/single_float"
        target_class, is_array = _get_field_type(self.example_topic_names_and_types, target)
        self.assertFalse(is_array)
        self.assertEqual(target_class, float)


def generate_test_get_field_type():
    """
    Utility for generating test_autogenerated_get_field_type.

    Run in the python interpreter and put the results in
    test_autogenerated_get_field_type
    """
    val_slots = ["", "floats", "unbounded_floats", "bounded_floats", "single_float"]
    val_slot_is_arr = [False, True, True, True, False]
    val_slot_class = ["Val", "float", "float", "float", "float"]

    array_val_slots = ["", "vals", "unbounded_vals", "bounded_vals", "single_val"]
    array_val_slot_is_arr = [False, True, True, True, False]
    array_val_slot_class = ["ArrayVal", "Val", "Val", "Val", "Val"]

    array_val_slots_len = len(array_val_slots)
    for i in [i for i in range(array_val_slots_len) if array_val_slots[i]]:
        for j, val_value in [(j, v) for (j, v) in enumerate(val_slots) if v]:
            if val_value:
                array_val_slots.append(array_val_slots[i] + "/" + val_value)
                array_val_slot_is_arr.append(val_slot_is_arr[j])
                array_val_slot_class.append(val_slot_class[j])

    indent = "    "
    print(indent + "def test_autogenerated_get_field_type(self):")
    print(indent * 2 + "'''")
    print(indent * 2 + "Test get field type for all of the example_topic_names_and_types")
    print("")
    print(indent * 2 + "These are generated by generate_test_get_field_type (below). In case the above")
    print(indent * 2 + "test fails these are useful because they have more useful test failure")
    print(indent * 2 + "information")
    print(indent * 2 + "'''")

    print(indent * 2 + "from rqt_py_common.topic_helpers import _get_field_type")
    print(indent * 2 + "from rqt_py_common.msg import ArrayVal, Val")
    for topic, msg_type in TestTopicHelpers.example_topic_names_and_types:
        slot_values = []
        slot_is_arr = []
        slot_class = []
        if msg_type[0] == 'rqt_py_common/Val':
            slot_values = val_slots
            slot_is_arr = val_slot_is_arr
            slot_class = val_slot_class
        elif msg_type[0] == 'rqt_py_common/ArrayVal':
            slot_values = array_val_slots
            slot_is_arr = array_val_slot_is_arr
            slot_class = array_val_slot_class

        for i, slot_value in enumerate(slot_values):
            target = topic + "/" + slot_value

            print("")
            print(indent * 2 + "target = \"%s\"" % target)
            # flake8 noqa
            print(indent * 2 + "target_class, is_array = _get_field_type(self.example_topic_names_and_types, target)")
            if slot_is_arr[i]:
                print(indent * 2 + "self.assertTrue(is_array)")
            else:
                print(indent * 2 + "self.assertFalse(is_array)")
            print(indent * 2 + "self.assertEqual(target_class, %s)" % slot_class[i])
