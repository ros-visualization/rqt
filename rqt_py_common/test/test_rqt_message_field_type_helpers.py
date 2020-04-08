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

import unittest


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
            ("/example[0]/field", ("/example[0]/field", False, -1)),
            ("/example/topic[]", ("", False, -1)),
            ("/example/topic[1]", ("/example/topic", True, 1)),
            ("/example[1]/topic[1]", ("/example[1]/topic", True, 1)),
            # We don't handle this case yet
            # ("/example[]/topic[1]", ("/example[]/topic", False, -1)),
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

    def test_get_field_type_array_information(self):
        from rqt_py_common.message_field_type_helpers import MessageFieldTypeInfo
        slot_type_to_info = {
            'boolean' : {
                'base_type_str': 'boolean', 'is_array': False,
                'is_static_array': False, 'static_array_size': -1,
                'is_bounded_array': False, 'bounded_array_size': -1,
                'is_unbounded_array': False,
                'is_bounded_string': False, 'bounded_string_size': -1},
            'octet' : {
                'base_type_str': 'octet', 'is_array': False,
                'is_static_array': False, 'static_array_size': -1,
                'is_bounded_array': False, 'bounded_array_size': -1,
                'is_unbounded_array': False,
                'is_bounded_string': False, 'bounded_string_size': -1},
            'float' : {
                'base_type_str': 'float', 'is_array': False,
                'is_static_array': False, 'static_array_size': -1,
                'is_bounded_array': False, 'bounded_array_size': -1,
                'is_unbounded_array': False,
                'is_bounded_string': False, 'bounded_string_size': -1},
            'double' : {
                'base_type_str': 'double', 'is_array': False,
                'is_static_array': False, 'static_array_size': -1,
                'is_bounded_array': False, 'bounded_array_size': -1,
                'is_unbounded_array': False,
                'is_bounded_string': False, 'bounded_string_size': -1},
            'uint8' : {
                'base_type_str': 'uint8', 'is_array': False,
                'is_static_array': False, 'static_array_size': -1,
                'is_bounded_array': False, 'bounded_array_size': -1,
                'is_unbounded_array': False,
                'is_bounded_string': False, 'bounded_string_size': -1},
            'int8' : {
                'base_type_str': 'int8', 'is_array': False,
                'is_static_array': False, 'static_array_size': -1,
                'is_bounded_array': False, 'bounded_array_size': -1,
                'is_unbounded_array': False,
                'is_bounded_string': False, 'bounded_string_size': -1},
            'int16' : {
                'base_type_str': 'int16', 'is_array': False,
                'is_static_array': False, 'static_array_size': -1,
                'is_bounded_array': False, 'bounded_array_size': -1,
                'is_unbounded_array': False,
                'is_bounded_string': False, 'bounded_string_size': -1},
            'int32' : {
                'base_type_str': 'int32', 'is_array': False,
                'is_static_array': False, 'static_array_size': -1,
                'is_bounded_array': False, 'bounded_array_size': -1,
                'is_unbounded_array': False,
                'is_bounded_string': False, 'bounded_string_size': -1},
            'int64' : {
                'base_type_str': 'int64', 'is_array': False,
                'is_static_array': False, 'static_array_size': -1,
                'is_bounded_array': False, 'bounded_array_size': -1,
                'is_unbounded_array': False,
                'is_bounded_string': False, 'bounded_string_size': -1},
            'uint8' : {
                'base_type_str': 'uint8', 'is_array': False,
                'is_static_array': False, 'static_array_size': -1,
                'is_bounded_array': False, 'bounded_array_size': -1,
                'is_unbounded_array': False,
                'is_bounded_string': False, 'bounded_string_size': -1},
            'uint16' : {
                'base_type_str': 'uint16', 'is_array': False,
                'is_static_array': False, 'static_array_size': -1,
                'is_bounded_array': False, 'bounded_array_size': -1,
                'is_unbounded_array': False,
                'is_bounded_string': False, 'bounded_string_size': -1},
            'uint32' : {
                'base_type_str': 'uint32', 'is_array': False,
                'is_static_array': False, 'static_array_size': -1,
                'is_bounded_array': False, 'bounded_array_size': -1,
                'is_unbounded_array': False,
                'is_bounded_string': False, 'bounded_string_size': -1},
            'uint64' : {
                'base_type_str': 'uint64', 'is_array': False,
                'is_static_array': False, 'static_array_size': -1,
                'is_bounded_array': False, 'bounded_array_size': -1,
                'is_unbounded_array': False,
                'is_bounded_string': False, 'bounded_string_size': -1},
            'string' : {
                'base_type_str': 'string', 'is_array': False,
                'is_static_array': False, 'static_array_size': -1,
                'is_bounded_array': False, 'bounded_array_size': -1,
                'is_unbounded_array': False,
                'is_bounded_string': False, 'bounded_string_size': -1},
            'int8[5]' : {
                'base_type_str': 'int8', 'is_array': True,
                'is_static_array': True, 'static_array_size': 5,
                'is_bounded_array': False, 'bounded_array_size': -1,
                'is_unbounded_array': False,
                'is_bounded_string': False, 'bounded_string_size': -1},
            'my_msgs/Wstring' : {
                'base_type_str': 'my_msgs/Wstring', 'is_array': False,
                'is_static_array': False, 'static_array_size': -1,
                'is_bounded_array': False, 'bounded_array_size': -1,
                'is_unbounded_array': False,
                'is_bounded_string': False, 'bounded_string_size': -1},
            'my_msgs/Wstring[1]' : {
                'base_type_str': 'my_msgs/Wstring', 'is_array': True,
                'is_static_array': True, 'static_array_size': 1,
                'is_bounded_array': False, 'bounded_array_size': -1,
                'is_unbounded_array': False,
                'is_bounded_string': False, 'bounded_string_size': -1},
            'sequence<my_msgs/Wstring>' : {
                'base_type_str': 'my_msgs/Wstring', 'is_array': True,
                'is_static_array': False, 'static_array_size': -1,
                'is_bounded_array': False, 'bounded_array_size': -1,
                'is_unbounded_array': True,
                'is_bounded_string': False, 'bounded_string_size': -1},
            'sequence<my_msgs/Wstring, 10>' : {
                'base_type_str': 'my_msgs/Wstring', 'is_array': True,
                'is_static_array': False, 'static_array_size': -1,
                'is_bounded_array': True, 'bounded_array_size': 10,
                'is_unbounded_array': False,
                'is_bounded_string': False, 'bounded_string_size': -1},
            'sequence<my_msgs/Wstring, 10>' : {
                'base_type_str': 'my_msgs/Wstring', 'is_array': True,
                'is_static_array': False, 'static_array_size': -1,
                'is_bounded_array': True, 'bounded_array_size': 10,
                'is_unbounded_array': False,
                'is_bounded_string': False, 'bounded_string_size': -1},
            'sequence<my_msgs/Wstring>' : {
                'base_type_str': 'my_msgs/Wstring', 'is_array': True,
                'is_static_array': False, 'static_array_size': -1,
                'is_bounded_array': False, 'bounded_array_size': -1,
                'is_unbounded_array': True,
                'is_bounded_string': False, 'bounded_string_size': -1},
            'sequence<int8>' : {
                'base_type_str': 'int8', 'is_array': True,
                'is_static_array': False, 'static_array_size': -1,
                'is_bounded_array': False, 'bounded_array_size': -1,
                'is_unbounded_array': True,
                'is_bounded_string': False, 'bounded_string_size': -1},
            'sequence<int8, 9>' : {
                'base_type_str': 'int8', 'is_array': True,
                'is_static_array': False, 'static_array_size': -1,
                'is_bounded_array': True, 'bounded_array_size': 9,
                'is_unbounded_array': False,
                'is_bounded_string': False, 'bounded_string_size': -1},
            'string[100]' : {
                'base_type_str': 'string', 'is_array': True,
                'is_static_array': True, 'static_array_size': 100,
                'is_bounded_array': False, 'bounded_array_size': -1,
                'is_unbounded_array': False,
                'is_bounded_string': False, 'bounded_string_size': -1},
            'sequence<string>' : {
                'base_type_str': 'string', 'is_array': True,
                'is_static_array': False, 'static_array_size': -1,
                'is_bounded_array': False, 'bounded_array_size': -1,
                'is_unbounded_array': True,
                'is_bounded_string': False, 'bounded_string_size': -1},
            'string<999>[34]' : {
                'base_type_str': 'string', 'is_array': True,
                'is_static_array': True, 'static_array_size': 34,
                'is_bounded_array': False, 'bounded_array_size': -1,
                'is_unbounded_array': False,
                'is_bounded_string': True, 'bounded_string_size': 999},
            'sequence<string<1>, 10>' : {
                'base_type_str': 'string', 'is_array': True,
                'is_static_array': False, 'static_array_size': -1,
                'is_bounded_array': True, 'bounded_array_size': 10,
                'is_unbounded_array': False,
                'is_bounded_string': True, 'bounded_string_size': 1},
            'sequence<string<40>>' : {
                'base_type_str': 'string', 'is_array': True,
                'is_static_array': False, 'static_array_size': -1,
                'is_bounded_array': False, 'bounded_array_size': -1,
                'is_unbounded_array': True,
                'is_bounded_string': True, 'bounded_string_size': 40},
            'sequence<string, 200>' : {
                'base_type_str': 'string', 'is_array': True,
                'is_static_array': False, 'static_array_size': -1,
                'is_bounded_array': True, 'bounded_array_size': 200,
                'is_unbounded_array': False,
                'is_bounded_string': False, 'bounded_string_size': -1},
            'string<2000>' : {
                'base_type_str': 'string', 'is_array': False,
                'is_static_array': False, 'static_array_size': -1,
                'is_bounded_array': False, 'bounded_array_size': -1,
                'is_unbounded_array': False,
                'is_bounded_string': True, 'bounded_string_size': 2000},
        }

        for slot, array_info in slot_type_to_info.items():
            generated_array_info = MessageFieldTypeInfo(slot)
            array_info_dict = generated_array_info.get_field_type_info_as_dict()
            for info_k, info_v in array_info.items():

                msg = "error on slot: [%s]" % slot
                self.assertTrue(info_k in array_info_dict.keys(), msg=msg + "array_info_dict does not have attribute: %s" % (info_k))
                self.assertTrue(hasattr(generated_array_info, info_k), msg=msg + "generated_array_info of type [%s] does not have attribute: %s" % (type(generated_array_info), info_k))

                self.assertEqual(info_v, array_info_dict[info_k], msg=msg + "array_info_dict[%s] != %s" % (info_k, info_v))
                self.assertEqual(info_v, getattr(generated_array_info, info_k), msg=msg + "generated_array_info[%s] != %s" % (info_k, info_v))


    def test_get_slot_class_and_field_information(self):
        from rqt_py_common.message_field_type_helpers import get_slot_class_and_field_information
        from rqt_py_common.msg import ArrayVal, Val

        val_slot_class = [Val, float, float, float, float, float, float, float]
        val_slot_array_info = {
             '' : {   # field_type = rqt_py_common/Val
                'base_type_str': 'rqt_py_common/Val', 'is_array': False,
                'is_static_array': False, 'static_array_size': -1,
                'is_bounded_array': False, 'bounded_array_size': -1,
                'is_unbounded_array': False,
                'is_bounded_string': False, 'bounded_string_size': -1, },
            'floats' : { # field_type = double[5]
                'base_type_str': 'double', 'is_array': True,
                'is_static_array': True, 'static_array_size': 5,
                'is_bounded_array': False, 'bounded_array_size': -1,
                'is_unbounded_array': False,
                'is_bounded_string': False, 'bounded_string_size': -1 },
            'floats[1]' : { # field_type = double[5]
                'base_type_str': 'double', 'is_array': False,
                'is_static_array': False, 'static_array_size': -1,
                'is_bounded_array': False, 'bounded_array_size': -1,
                'is_unbounded_array': False,
                'is_bounded_string': False, 'bounded_string_size': -1 },
            'unbounded_floats' : { # field_type = sequence<double>
                'base_type_str': 'double', 'is_array': True,
                'is_static_array': False, 'static_array_size': -1,
                'is_bounded_array': False, 'bounded_array_size': -1,
                'is_unbounded_array': True,
                'is_bounded_string': False, 'bounded_string_size': -1 },
            'unbounded_floats[0]' : { # field_type = sequence<double>
                'base_type_str': 'double', 'is_array': False,
                'is_static_array': False, 'static_array_size': -1,
                'is_bounded_array': False, 'bounded_array_size': -1,
                'is_unbounded_array': False,
                'is_bounded_string': False, 'bounded_string_size': -1 },
            'bounded_floats' : { # field_type = sequence<double, 3>
                'base_type_str': 'double', 'is_array': True,
                'is_static_array': False, 'static_array_size': -1,
                'is_bounded_array': True, 'bounded_array_size': 3,
                'is_unbounded_array': False,
                'is_bounded_string': False, 'bounded_string_size': -1 },
            'bounded_floats[0]' : { # field_type = sequence<double, 3>
                'base_type_str': 'double', 'is_array': False,
                'is_static_array': False, 'static_array_size': -1,
                'is_bounded_array': False, 'bounded_array_size': -1,
                'is_unbounded_array': False,
                'is_bounded_string': False, 'bounded_string_size': -1 },
            'single_float' : { # field_type = double
                'base_type_str': 'double', 'is_array': False,
                'is_static_array': False, 'static_array_size': -1,
                'is_bounded_array': False, 'bounded_array_size': -1,
                'is_unbounded_array': False,
                'is_bounded_string': False, 'bounded_string_size': -1 }}

        array_val_slot_class = [ArrayVal, Val, Val, Val, Val, Val, Val, Val]
        array_val_slot_array_info = {
            '' : {   # ield_type = rqt_py_common/ArrayVal
                'base_type_str': 'rqt_py_common/ArrayVal', 'is_array': False,
                'is_static_array': False, 'static_array_size': -1,
                'is_bounded_array': False, 'bounded_array_size': -1,
                'is_unbounded_array': False,
                'is_bounded_string': False, 'bounded_string_size': -1, },
            'vals' : { # field_type = rqt_py_common/Val[5]
                'base_type_str': 'rqt_py_common/Val', 'is_array': True,
                'is_static_array': True, 'static_array_size': 5,
                'is_bounded_array': False, 'bounded_array_size': -1,
                'is_unbounded_array': False,
                'is_bounded_string': False, 'bounded_string_size': -1 },
            'vals[3]' : { # field_type = rqt_py_common/Val[5]
                'base_type_str': 'rqt_py_common/Val', 'is_array': False,
                'is_static_array': False, 'static_array_size': -1,
                'is_bounded_array': False, 'bounded_array_size': -1,
                'is_unbounded_array': False,
                'is_bounded_string': False, 'bounded_string_size': -1 },
            'unbounded_vals' : { # field_type = sequence<rqt_py_common/Val>
                'base_type_str': 'rqt_py_common/Val', 'is_array': True,
                'is_static_array': False, 'static_array_size': -1,
                'is_bounded_array': False, 'bounded_array_size': -1,
                'is_unbounded_array': True,
                'is_bounded_string': False, 'bounded_string_size': -1 },
            'unbounded_vals[0]' : { # field_type = sequence<rqt_py_common/Val>
                'base_type_str': 'rqt_py_common/Val', 'is_array': False,
                'is_static_array': False, 'static_array_size': -1,
                'is_bounded_array': False, 'bounded_array_size': -1,
                'is_unbounded_array': False,
                'is_bounded_string': False, 'bounded_string_size': -1 },
            'bounded_vals' : { # field_type = sequence<rqt_py_common/Val, 5>
                'base_type_str': 'rqt_py_common/Val', 'is_array': True,
                'is_static_array': False, 'static_array_size': -1,
                'is_bounded_array': True, 'bounded_array_size': 5,
                'is_unbounded_array': False,
                'is_bounded_string': False, 'bounded_string_size': -1 },
            'bounded_vals[2]' : { # field_type = sequence<rqt_py_common/Val, 5>
                'base_type_str': 'rqt_py_common/Val', 'is_array': False,
                'is_static_array': False, 'static_array_size': -1,
                'is_bounded_array': False, 'bounded_array_size': -1,
                'is_unbounded_array': False,
                'is_bounded_string': False, 'bounded_string_size': -1 },
            'single_val' : { # field_type = rqt_py_common/Val
                'base_type_str': 'rqt_py_common/Val', 'is_array': False,
                'is_static_array': False, 'static_array_size': -1,
                'is_bounded_array': False, 'bounded_array_size': -1,
                'is_unbounded_array': False,
                'is_bounded_string': False, 'bounded_string_size': -1 }}


        array_val_slots = list(array_val_slot_array_info.keys())
        array_val_slots_len = len(array_val_slots)
        for i in [i for i in range(array_val_slots_len) if array_val_slots[i]]:
            for j, val_value in [(j, v) for (j, v) in enumerate(val_slot_array_info.keys()) if v]:
                if val_value:
                    key = array_val_slots[i] + '/' + val_value
                    array_val_slot_array_info[key] = val_slot_array_info[val_value]
                    array_val_slot_class.append(val_slot_class[j])

        for msg_type in ['rqt_py_common/Val', 'rqt_py_common/ArrayVal']:
            top_level_class = None
            slot_values = []
            slot_array_info = {}
            slot_class = []
            if msg_type == 'rqt_py_common/Val':
                top_level_class = Val
                slot_values = list(val_slot_array_info.keys())
                slot_array_info = val_slot_array_info
                slot_class = val_slot_class
            elif msg_type == 'rqt_py_common/ArrayVal':
                top_level_class = ArrayVal
                slot_values = list(array_val_slot_array_info.keys())
                slot_array_info = array_val_slot_array_info
                slot_class = array_val_slot_class

            for i, slot_value in enumerate(slot_values):
                target = "/" + slot_value
                target_class, field_info = get_slot_class_and_field_information(
                    top_level_class, slot_value
                )

                if target_class != slot_class[i]:
                    error

                field_info_dict = field_info.get_field_type_info_as_dict()

                base_msg = '[top_level_class = "%s", slot_value = "%s"]\t' % (top_level_class, slot_value)

                self.assertTrue(target_class is not None, msg=base_msg + '"%s" is None' % target_class)
                self.assertTrue(field_info is not None, msg=base_msg + '"%s" is None' % field_info)
                self.assertEqual(target_class, slot_class[i])
                for info_k, info_v in slot_array_info[slot_value].items():
                    self.assertTrue(
                        info_k in field_info_dict,
                        msg=base_msg + '"%s" not in "%s"' % (
                            info_k, field_info_dict
                        )
                    )
                    self.assertEqual(
                        info_v, field_info_dict[info_k],
                        msg=base_msg + \
                            '"%s" != field_info_dict[%s] ie: "%s"' % (
                                info_v,
                                info_k,
                                field_info_dict[info_k]
                            )
                    )

    def test_get_base_python_type(self):  # noqa: D102
        from rqt_py_common.message_field_type_helpers import get_base_python_type
        from rqt_py_common.message_helpers import get_message_class
        from rqt_py_common.msg import ArrayVal, Val

        field_type_to_python_type_map = {
            'boolean'                 : bool,
            'octet'                   : bytes,
            'float'                   : float,
            'double'                  : float,
            'uint8'                   : int,
            'int8'                    : int,
            'int16'                   : int,
            'int32'                   : int,
            'int64'                   : int,
            'uint8'                   : int,
            'uint16'                  : int,
            'uint32'                  : int,
            'uint64'                  : int,
            'string'                  : str,
            'rqt_py_common/ArrayVal'  : ArrayVal,
            'rqt_py_common/Val'       : Val,
            'int8[3]'                 : int,
            'sequence<int8>'          : int,
            'sequence<int8, 3>'       : int,
            'string<5>'               : str,
            'string[3]'               : str,
            'sequence<string>'        : str,
            'string<5>[3]'            : str,
            'sequence<string<5>, 10>' : str,
            'sequence<string<5>>'     : str,
            'sequence<string, 10>'    : str,
        }

        for k, v in field_type_to_python_type_map.items():
            self.assertEqual(
                get_base_python_type(k),
                v,
                msg='get_base_python_type(\'%s\') != \'%s\'' % (k, v))

        message_type_str = 'float[]'
        python_type = get_base_python_type(message_type_str)
        self.assertEqual(
            python_type, float,
            msg='python_type (\'%s\') != \'%s\'' % (python_type, str(float)))

        python_type_str = 'rqt_py_common/ArrayVal'
        python_type = get_base_python_type(python_type_str)
