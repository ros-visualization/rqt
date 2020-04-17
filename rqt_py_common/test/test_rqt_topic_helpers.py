#!/usr/bin/env python3

# Software License Agreement (BSD License)
#
# Copyright (c) 2020, Michael Lautman, PickNik Robotics
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
from rqt_py_common.msg import ArrayVal, Val

def _get_val_data():
    val_slots = ['', 'floats', 'unbounded_floats', 'bounded_floats', 'single_float']
    val_slot_is_arr = [False, True, True, True, False]
    val_slot_class = [Val, float, float, float, float]
    return val_slots, val_slot_is_arr, val_slot_class

def _get_arr_val_data():
    val_slots, val_slot_is_arr, val_slot_class = _get_val_data()
    array_val_slots = ['', 'vals', 'unbounded_vals', 'bounded_vals', 'single_val']
    array_val_slot_is_arr = [False, True, True, True, False]
    array_val_slot_class = [ArrayVal, Val, Val, Val, Val]

    array_val_slots_len = len(array_val_slots)
    for i in [i for i in range(array_val_slots_len) if array_val_slots[i]]:
        for j, val_value in [(j, v) for (j, v) in enumerate(val_slots) if v]:
            if val_value:
                array_val_slots.append(array_val_slots[i] + '/' + val_value)
                array_val_slot_is_arr.append(val_slot_is_arr[j])
                array_val_slot_class.append(val_slot_class[j])

    return array_val_slots, array_val_slot_is_arr, array_val_slot_class

class TestTopicHelpers(unittest.TestCase):  # noqa: D101
    example_topic_names_and_types = [
        ('/example', ['rqt_py_common/Val']),
        ('/example/vals', ['rqt_py_common/Val']),
        ('/example_topic', ['rqt_py_common/Val']),
        ('/example/topic', ['rqt_py_common/ArrayVal']),
    ]


    def test_separate_field_from_array_information(self):
        from rqt_py_common.message_field_type_helpers import \
            separate_field_from_array_information

        test_values = [
            ("/example[0]/field", ("/example[0]/field", False, None)),
            ("/example/topic[]", ("", False, None)),
            ("/example/topic[1]", ("/example/topic", True, 1)),
            ("/example[1]/topic[1]", ("/example[1]/topic", True, 1)),
            # We don't handle this case yet
            # ("/example[]/topic[1]", ("/example[]/topic", False, None)),
        ]

        for test_input, expected_test_output in test_values:
            test_output = separate_field_from_array_information(test_input)

            msg = "error on test_input: [%s]\t" % test_input
            self.assertEqual(
                len(test_output), 3,
                msg=msg + "len(%s) != 3" % len(test_output))
            for i, v in enumerate(test_output):
                self.assertEqual(
                    v, expected_test_output[i],
                    msg=msg + 'test_output[%d]: "%s" != "%s"' % (
                        i, v, expected_test_output
                    )
                )
    def test_get_field_type(self):  # noqa: D102
        from rqt_py_common.topic_helpers import _get_field_type

        val_slots, val_slot_is_arr, val_slot_class = _get_val_data()
        array_val_slots, array_val_slot_is_arr, array_val_slot_class = \
            _get_arr_val_data()

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

                msg = "\n---\ntopic = [%s]\nmsg_type = [%s]\ntarget: [%s]\n---\n" % (
                    topic, msg_type[0], target)

                self.assertEqual(slot_is_arr[i], is_array,
                    msg=msg + "\tslot_is_arr[%s]: %s != %s" % (i, slot_is_arr[i], is_array))
                self.assertEqual(slot_class[i], target_class,
                    msg=msg + "\tslot_class[%s]: '%s' != '%s'" % (i, slot_class[i], target_class))

    def test_autogenerated_get_field_type(self):
        """
        Test get field type for all of the example_topic_names_and_types

        These are generated by generate_test_get_field_type (below). In case the above
        test fails these are useful because they have more useful test failure
        information
        """
        from rqt_py_common.topic_helpers import _get_field_type

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
    val_slots, val_slot_is_arr, val_slot_class = _get_val_data()
    array_val_slots, array_val_slot_is_arr, array_val_slot_class = _get_arr_val_data()

    indent = '    '
    print(indent + 'def test_autogenerated_get_field_type(self):')
    print(indent * 2 + '"""')
    print(indent * 2 + 'Test get field type for all of the example_topic_names_and_types')
    print('')
    print(indent * 2 + 'These are generated by generate_test_get_field_type (below). In case the above')
    print(indent * 2 + 'test fails these are useful because they have more useful test failure')
    print(indent * 2 + 'information')
    print(indent * 2 + '"""')

    print(indent * 2 + 'from rqt_py_common.topic_helpers import _get_field_type')
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
            target = topic + '/' + slot_value

            print('')
            print(indent * 2 + 'target = "%s"' % target)
            # flake8 noqa
            print(indent * 2 + 'target_class, is_array = _get_field_type(self.example_topic_names_and_types, target)')
            if slot_is_arr[i]:
                print(indent * 2 + 'self.assertTrue(is_array)')
            else:
                print(indent * 2 + 'self.assertFalse(is_array)')
            print(indent * 2 + 'self.assertEqual(target_class, %s)' % slot_class[i])
