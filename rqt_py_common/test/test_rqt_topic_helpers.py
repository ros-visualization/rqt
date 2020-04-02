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

        val_slots =["", "floats", "unbounded_floats", "bounded_floats", "single_float"]
        val_slot_is_arr =[False, True, True, True, False]
        val_slot_class =[Val, float, float, float, float]

        array_val_slots = ["", "vals", "unbounded_vals", "bounded_vals", "single_val"]
        array_val_slot_is_arr =[False, True, True, True, False]
        array_val_slot_class =[ArrayVal, Val, Val, Val, Val]

        array_val_slots_len = len(array_val_slots)
        for i in [i for i in range(array_val_slots_len) if array_val_slots[i]]:
                for j, val_value in [(j, v) for (j, v) in enumerate(val_slots) if v]:
                    if val_value:
                        array_val_slots.append(array_val_slots[i] + "/" + val_value)
                        array_val_slot_is_arr.append(val_slot_is_arr[j])
                        array_val_slot_class.append(val_slot_class[j])

        for topic, msg_type in self.example_topic_names_and_types:
            print("On topic: ", topic)
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
                print("On target: ", target)
                target_class, is_array = _get_field_type(self.example_topic_names_and_types, target)

                print("target", target, "target_class", target_class, "slot_class[i]", slot_class[i], "is_arr", is_array, 'slot_is_arr[i]', slot_is_arr[i])

                if slot_is_arr[i]:
                    self.assertTrue(is_array)
                else:
                    self.assertFalse(is_array)
                self.assertEqual(target_class, slot_class[i])

    def test_autogenerated_get_field_type(self):
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
    from rqt_py_common.topic_helpers import _get_field_type
    from rqt_py_common.msg import ArrayVal, Val

    val_slots =["", "floats", "unbounded_floats", "bounded_floats", "single_float"]
    val_slot_is_arr =[False, True, True, True, False]
    val_slot_class =["Val", "float", "float", "float", "float"]

    array_val_slots = ["", "vals", "unbounded_vals", "bounded_vals", "single_val"]
    array_val_slot_is_arr =[False, True, True, True, False]
    array_val_slot_class =["ArrayVal", "Val", "Val", "Val", "Val"]

    array_val_slots_len = len(array_val_slots)
    for i in [i for i in range(array_val_slots_len) if array_val_slots[i]]:
            for j, val_value in [(j, v) for (j, v) in enumerate(val_slots) if v]:
                if val_value:
                    array_val_slots.append(array_val_slots[i] + "/" + val_value)
                    array_val_slot_is_arr.append(val_slot_is_arr[j])
                    array_val_slot_class.append(val_slot_class[j])

    indent = "    "
    print(indent + "def test_autogenerated_get_field_type(self):")
    print(indent * 2 + "from rqt_py_common.topic_helpers import _get_field_type")
    print(indent * 2 + "from rqt_py_common.msg import ArrayVal, Val")
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

            print("")
            print(indent * 2 + "target = \"%s\"" % target)
            print(indent * 2 + "target_class, is_array = _get_field_type(self.example_topic_names_and_types, target)")
            if slot_is_arr[i]:
                print(indent * 2 + "self.assertTrue(is_array)")
            else:
                print(indent * 2 + "self.assertFalse(is_array)")
            print(indent * 2 + "self.assertEqual(target_class, %s)" % slot_class[i])