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
from copy import deepcopy
from rqt_py_common.msg import ArrayVal, Val


class TestTopicHelpers(unittest.TestCase):  # noqa: D101
    non_bounded_string_types = [
        'boolean', 'octet', 'float', 'double', 'uint8', 'int8', 'int16',
        'int32', 'int64', 'uint8', 'uint16', 'uint32', 'uint64',
        'my_msgs/Wstring', 'my_msgs/String', 'my_msgs/MyString',
        'string', 'wstring']

    string_types = ['string', 'wstring']



    val_slot_class = [Val, float, float, float, float, float, float, float]
    val_slot_array_info = {
         '' : {   # field_type = rqt_py_common/Val
            'is_valid' : True,
            'base_type_str': 'rqt_py_common/Val', 'is_array': False,
            'is_static_array': False, 'static_array_size': None,
            'is_bounded_array': False, 'bounded_array_size': None,
            'is_unbounded_array': False,
            'is_bounded_string': False, 'bounded_string_size': None, },
        'floats' : { # field_type = double[5]
            'is_valid' : True,
            'base_type_str': 'double', 'is_array': True,
            'is_static_array': True, 'static_array_size': 5,
            'is_bounded_array': False, 'bounded_array_size': None,
            'is_unbounded_array': False,
            'is_bounded_string': False, 'bounded_string_size': None },
        'floats[1]' : { # field_type = double[5]
            'is_valid' : True,
            'base_type_str': 'double', 'is_array': False,
            'is_static_array': False, 'static_array_size': None,
            'is_bounded_array': False, 'bounded_array_size': None,
            'is_unbounded_array': False,
            'is_bounded_string': False, 'bounded_string_size': None },
        'unbounded_floats' : { # field_type = sequence<double>
            'is_valid' : True,
            'base_type_str': 'double', 'is_array': True,
            'is_static_array': False, 'static_array_size': None,
            'is_bounded_array': False, 'bounded_array_size': None,
            'is_unbounded_array': True,
            'is_bounded_string': False, 'bounded_string_size': None },
        'unbounded_floats[0]' : { # field_type = sequence<double>
            'is_valid' : True,
            'base_type_str': 'double', 'is_array': False,
            'is_static_array': False, 'static_array_size': None,
            'is_bounded_array': False, 'bounded_array_size': None,
            'is_unbounded_array': False,
            'is_bounded_string': False, 'bounded_string_size': None },
        'bounded_floats' : { # field_type = sequence<double, 3>
            'is_valid' : True,
            'base_type_str': 'double', 'is_array': True,
            'is_static_array': False, 'static_array_size': None,
            'is_bounded_array': True, 'bounded_array_size': 3,
            'is_unbounded_array': False,
            'is_bounded_string': False, 'bounded_string_size': None },
        'bounded_floats[0]' : { # field_type = sequence<double, 3>
            'is_valid' : True,
            'base_type_str': 'double', 'is_array': False,
            'is_static_array': False, 'static_array_size': None,
            'is_bounded_array': False, 'bounded_array_size': None,
            'is_unbounded_array': False,
            'is_bounded_string': False, 'bounded_string_size': None },
        'single_float' : { # field_type = double
            'is_valid' : True,
            'base_type_str': 'double', 'is_array': False,
            'is_static_array': False, 'static_array_size': None,
            'is_bounded_array': False, 'bounded_array_size': None,
            'is_unbounded_array': False,
            'is_bounded_string': False, 'bounded_string_size': None }}

    array_val_slot_class = [ArrayVal, Val, Val, Val, Val, Val, Val, Val]
    array_val_slot_array_info = {
        '' : {   # ield_type = rqt_py_common/ArrayVal
            'is_valid' : True,
            'base_type_str': 'rqt_py_common/ArrayVal', 'is_array': False,
            'is_static_array': False, 'static_array_size': None,
            'is_bounded_array': False, 'bounded_array_size': None,
            'is_unbounded_array': False,
            'is_bounded_string': False, 'bounded_string_size': None, },
        'vals' : { # field_type = rqt_py_common/Val[5]
            'is_valid' : True,
            'base_type_str': 'rqt_py_common/Val', 'is_array': True,
            'is_static_array': True, 'static_array_size': 5,
            'is_bounded_array': False, 'bounded_array_size': None,
            'is_unbounded_array': False,
            'is_bounded_string': False, 'bounded_string_size': None },
        'vals[3]' : { # field_type = rqt_py_common/Val[5]
            'is_valid' : True,
            'base_type_str': 'rqt_py_common/Val', 'is_array': False,
            'is_static_array': False, 'static_array_size': None,
            'is_bounded_array': False, 'bounded_array_size': None,
            'is_unbounded_array': False,
            'is_bounded_string': False, 'bounded_string_size': None },
        'unbounded_vals' : { # field_type = sequence<rqt_py_common/Val>
            'is_valid' : True,
            'base_type_str': 'rqt_py_common/Val', 'is_array': True,
            'is_static_array': False, 'static_array_size': None,
            'is_bounded_array': False, 'bounded_array_size': None,
            'is_unbounded_array': True,
            'is_bounded_string': False, 'bounded_string_size': None },
        'unbounded_vals[0]' : { # field_type = sequence<rqt_py_common/Val>
            'is_valid' : True,
            'base_type_str': 'rqt_py_common/Val', 'is_array': False,
            'is_static_array': False, 'static_array_size': None,
            'is_bounded_array': False, 'bounded_array_size': None,
            'is_unbounded_array': False,
            'is_bounded_string': False, 'bounded_string_size': None },
        'bounded_vals' : { # field_type = sequence<rqt_py_common/Val, 5>
            'is_valid' : True,
            'base_type_str': 'rqt_py_common/Val', 'is_array': True,
            'is_static_array': False, 'static_array_size': None,
            'is_bounded_array': True, 'bounded_array_size': 5,
            'is_unbounded_array': False,
            'is_bounded_string': False, 'bounded_string_size': None },
        'bounded_vals[2]' : { # field_type = sequence<rqt_py_common/Val, 5>
            'is_valid' : True,
            'base_type_str': 'rqt_py_common/Val', 'is_array': False,
            'is_static_array': False, 'static_array_size': None,
            'is_bounded_array': False, 'bounded_array_size': None,
            'is_unbounded_array': False,
            'is_bounded_string': False, 'bounded_string_size': None },
        'single_val' : { # field_type = rqt_py_common/Val
            'is_valid' : True,
            'base_type_str': 'rqt_py_common/Val', 'is_array': False,
            'is_static_array': False, 'static_array_size': None,
            'is_bounded_array': False, 'bounded_array_size': None,
            'is_unbounded_array': False,
            'is_bounded_string': False, 'bounded_string_size': None }}


    def test_get_field_type_array_information(self):
        from rqt_py_common.message_field_type_helpers import MessageFieldTypeInfo

        non_string_slot_type_info = {
            '%s' : {
                'is_valid': True,
                'field_type_without_array_info': '%s',
                'base_type_str': '%s', 'is_array': False,
                'is_static_array': False, 'static_array_size': None,
                'is_bounded_array': False, 'bounded_array_size': None,
                'is_unbounded_array': False,
                'is_bounded_string': False, 'bounded_string_size': None},
            '%s[5]' : {
                'is_valid': True,
                'field_type_without_array_info': '%s',
                'base_type_str': '%s', 'is_array': True,
                'is_static_array': True, 'static_array_size': 5,
                'is_bounded_array': False, 'bounded_array_size': None,
                'is_unbounded_array': False,
                'is_bounded_string': False, 'bounded_string_size': None},
            'sequence<%s>' : {
                'is_valid': True,
                'field_type_without_array_info': '%s',
                'base_type_str': '%s', 'is_array': True,
                'is_static_array': False, 'static_array_size': None,
                'is_bounded_array': False, 'bounded_array_size': None,
                'is_unbounded_array': True,
                'is_bounded_string': False, 'bounded_string_size': None},
            'sequence<%s, 10>' : {
                'is_valid': True,
                'field_type_without_array_info': '%s',
                'base_type_str': '%s', 'is_array': True,
                'is_static_array': False, 'static_array_size': None,
                'is_bounded_array': True, 'bounded_array_size': 10,
                'is_unbounded_array': False,
                'is_bounded_string': False, 'bounded_string_size': None},
            'sequence<%s, 100>' : {
                'is_valid': True,
                'field_type_without_array_info': '%s',
                'base_type_str': '%s', 'is_array': True,
                'is_static_array': False, 'static_array_size': None,
                'is_bounded_array': True, 'bounded_array_size': 10,
                'is_unbounded_array': False,
                'is_bounded_string': False, 'bounded_string_size': None},
            'sequence<%s, 1>' : {
                'is_valid': True,
                'field_type_without_array_info': '%s',
                'base_type_str': '%s', 'is_array': True,
                'is_static_array': False, 'static_array_size': None,
                'is_bounded_array': True, 'bounded_array_size': 10,
                'is_unbounded_array': False,
                'is_bounded_string': False, 'bounded_string_size': None},
        }

        slot_types = TestTopicHelpers.non_bounded_string_types
        slot_type_infos = non_string_slot_type_info
        for slot_type in slot_types:
            for slot_pattern, array_info_pattern in slot_type_infos.items():
                # Make copies
                slot = slot_pattern
                array_info = deepcopy(array_info_pattern)

                # String Substitution
                slot = slot_pattern % slot_type
                array_info['base_type_str'] = array_info['base_type_str'] % slot_type
                array_info['field_type_without_array_info'] = \
                    array_info['field_type_without_array_info'] % slot_type

                gen_arr_info = MessageFieldTypeInfo(slot)
                gen_arr_info_dict = gen_arr_info.to_dict()
                for info_k, info_v in array_info.items():
                    msg = "Error on slot: [%s]" % slot

                    self.assertTrue(
                        info_k in gen_arr_info_dict.keys(),
                        msg=msg + "attribute: [%s] not in 'gen_arr_info_dict.keys()':\n%s" % (
                            info_k, gen_arr_info_dict.keys()))
                    self.assertEqual(
                        info_v, gen_arr_info_dict[info_k],
                        msg=msg + "gen_arr_info_dict[%s]: '%s' != '%s'" % (
                            info_k, gen_arr_info_dict[info_k], info_v))

                    self.assertTrue(
                        hasattr(gen_arr_info, info_k),
                        msg=msg + "gen_arr_info of type [%s] does not have attribute: %s" % (
                            type(gen_arr_info), info_k))
                    self.assertEqual(
                        info_v,
                        getattr(gen_arr_info, info_k),
                        msg=msg + "gen_arr_info[%s]: '%s' != '%s'" % (
                            info_k, gen_arr_info[info_k], info_v))

    def test_get_field_type_array_information(self):
        from rqt_py_common.message_field_type_helpers import MessageFieldTypeInfo

        bounded_string_slot_type_info = {
            '%s<999>[34]' : {
                'is_valid': True,
                'field_type_without_array_info': '%s<999>',
                'base_type_str': '%s', 'is_array': True,
                'is_static_array': True, 'static_array_size': 34,
                'is_bounded_array': False, 'bounded_array_size': None,
                'is_unbounded_array': False,
                'is_bounded_string': True, 'bounded_string_size': 999},
            'sequence<%s<1>, 10>' : {
                'is_valid': True,
                'field_type_without_array_info': '%s<1>',
                'base_type_str': '%s', 'is_array': True,
                'is_static_array': False, 'static_array_size': None,
                'is_bounded_array': True, 'bounded_array_size': 10,
                'is_unbounded_array': False,
                'is_bounded_string': True, 'bounded_string_size': 1},
            'sequence<%s<40>>' : {
                'is_valid': True,
                'field_type_without_array_info': '%s<40>',
                'base_type_str': '%s', 'is_array': True,
                'is_static_array': False, 'static_array_size': None,
                'is_bounded_array': False, 'bounded_array_size': None,
                'is_unbounded_array': True,
                'is_bounded_string': True, 'bounded_string_size': 40},
            '%s<2000>' : {
                'is_valid': True,
                'field_type_without_array_info': '%s<2000>',
                'base_type_str': '%s', 'is_array': False,
                'is_static_array': False, 'static_array_size': None,
                'is_bounded_array': False, 'bounded_array_size': None,
                'is_unbounded_array': False,
                'is_bounded_string': True, 'bounded_string_size': 2000},
        }

        for slot_type in TestTopicHelpers.string_types:
            for slot_pattern, array_info_pattern in bounded_string_slot_type_info.items():
                # Make copies
                slot = slot_pattern
                array_info = deepcopy(array_info_pattern)
                # String Substitution
                slot = slot_pattern % slot_type
                array_info['base_type_str'] = array_info['base_type_str'] % slot_type
                array_info['field_type_without_array_info'] = \
                    array_info['field_type_without_array_info'] % slot_type

                gen_arr_info = MessageFieldTypeInfo(slot)
                gen_arr_info_dict = gen_arr_info.to_dict()
                for info_k, info_v in array_info.items():

                    msg = "Error on slot: [%s]" % slot
                    self.assertTrue(
                        info_k in gen_arr_info_dict.keys(),
                        msg=msg + "attribute: [%s] not in 'gen_arr_info_dict.keys()':\n%s" % (
                            info_k, gen_arr_info_dict.keys()))
                    self.assertEqual(
                        info_v, gen_arr_info_dict[info_k],
                        msg=msg + "gen_arr_info_dict[%s]: '%s' != '%s'" % (
                            info_k, gen_arr_info_dict[info_k], info_v))

                    self.assertTrue(
                        hasattr(gen_arr_info, info_k),
                        msg=msg + "gen_arr_info of type [%s] does not have attribute: %s" % (
                            type(gen_arr_info), info_k))
                    self.assertEqual(
                        info_v,
                        getattr(gen_arr_info, info_k),
                        msg=msg + "gen_arr_info[%s]: '%s' != '%s'" % (
                            info_k, gen_arr_info, info_v))

    def generate_invalid_format_list_and_arr_info_dict(self, bounded_string = False):
        # Some degenerates
        arr_info_dict_for_invalid = {
                'is_valid': False,
                'base_type_str': '', 'is_array': False,
                'is_static_array': False, 'static_array_size': None,
                'is_bounded_array': False, 'bounded_array_size': None,
                'is_unbounded_array': False,
                'is_bounded_string': False, 'bounded_string_size': None}
        invalid_sequence_formats = [
            'sequence<%s>[1]',
            'sequence<%s, 1>[1]',
            'sequence<<%s>>',
            'sequence<<%s>',
            'sequence<%s>>',
            'sequence%s>',
            'sequence<%s',
            'sequence<%s >',
            'sequence< %s>',
            'sequence<%s>a',
            'ssequence<%s>',
            'sequence<<%s, 3>>',
            'sequence<<%s, 3>',
            'sequence<%s, 3>>',
            'sequence%s, 3>',
            'sequence<%s, 3',
            'sequence%s, 3',
            'sequence<%s, 3 >',
            'sequence< %s, 3>',
            'sequence<%s, 3>a',
            'ssequence<%s, 3>',
            'sequence<%s, >',
            'sequence<%s 3>',
            'sequence<%s, 3.>',
            'sequence<%s, 3.1>',
            'sequence<%s, .1>',
            'sequence<<%s, 0>',
            'sequence<<%s, 3>>',
            'sequence<%s, 3>>',
            'sequence%s, 3>',
            'sequence<%s, 3',
            'sequence%s, 3',
            'sequence<%s , 3>',
            'sequence<%s,  3>',
            'sequencee<%s, 3>',
            'sequence<%s, 3>a',
            'sequence<%s, 3.a>',
        ]

        patterns = [
            '', '{0}1{1}', '10', # valid on their own
            '{0}{1}', '10{1}', '{0}10', # invalid
            '{0}{0}10{1}', '{0}10{1}{1}', '{0}{0}10{1}{1}',
            '{0}0{1}', '{0}0.{1}', '{0}1.1{1}', '{0}1.{1}{1}', '{0}{0}0.0{1}{1}',
            '{0}a{1}', '{0}a.{1}', '{0}1.a{1}', '{0}a.{1}{1}', '{0}{0}a.a{1}{1}',
            '{0}1a{1}', '{0}1 {1}', '{0} 1{1}']

        static_array_err = []
        bounded_string_err = []
        for pattern in patterns:
            static_array_err.append(pattern.format('[', ']'))
            bounded_string_err.append(pattern.format('<', '>'))

        invalid_sequence_formats.extend(['%s' + sae for sae in static_array_err[3:]])

        invalid_format_list = []
        if bounded_string:
            for i, bse in enumerate(bounded_string_err):
                for j, sae in enumerate(static_array_err):
                    if not (i < 3 and j < 3):
                        invalid_format_list.append('%s' + bse + sae)
        else:
            for j, sae in enumerate(static_array_err):
                if j > 3:
                    invalid_format_list.append('%s' + sae)

        return arr_info_dict_for_invalid, invalid_format_list


    def test_get_field_type_array_information_not_valid(self):
        from rqt_py_common.message_field_type_helpers import MessageFieldTypeInfo

        arr_info_dict_for_invalid, invalid_format_list = \
            self.generate_invalid_format_list_and_arr_info_dict(bounded_string=True)

        for i, bad_slot in enumerate(invalid_format_list):
            for base_type in TestTopicHelpers.non_bounded_string_types:
                bad_slot_tmp = bad_slot % base_type

                gen_arr_info = MessageFieldTypeInfo(bad_slot_tmp)
                gen_arr_info_dic = gen_arr_info.to_dict()

                for info_k, info_v in arr_info_dict_for_invalid.items():
                    msg = "error on bad_slot[%s]: '%s' -> " % (i, bad_slot)
                    self.assertTrue(
                        info_k in gen_arr_info_dic.keys(),
                        msg=msg + "attribute: [%s] not in gen_arr_info_dic.keys():\n%s" % (
                            info_k, repr(gen_arr_info_dic.keys())))
                    self.assertEqual(
                        info_v, gen_arr_info_dic[info_k],
                        msg=msg + "gen_arr_info_dic[%s]: '%s' != '%s'" % (
                            info_k, gen_arr_info_dic[info_k], info_v))


                    self.assertTrue(
                        hasattr(gen_arr_info, info_k),
                        msg=msg + "gen_arr_info of type [%s] does not have attribute: [%s]" % (
                            type(gen_arr_info), info_k))
                    self.assertEqual(
                        info_v, getattr(gen_arr_info, info_k),
                        msg=msg + "gen_arr_info[%s]: '%s' != '%s'" % (
                            info_k, getattr(gen_arr_info, info_k), info_v))

    def test_get_field_type_array_information_not_valid_strings(self):
        from rqt_py_common.message_field_type_helpers import MessageFieldTypeInfo


        arr_info_dict_for_invalid, invalid_format_list = self.generate_invalid_format_list_and_arr_info_dict()

        for bad_slot in invalid_format_list:
            for base_type in TestTopicHelpers.string_types:
                bad_slot_tmp = bad_slot % base_type

                gen_arr_info = MessageFieldTypeInfo(bad_slot_tmp)
                gen_arr_info_dic = gen_arr_info.to_dict()

                for info_k, info_v in arr_info_dict_for_invalid.items():
                    msg = "error on bad_slot: '%s' -> " % bad_slot_tmp
                    self.assertTrue(
                        info_k in gen_arr_info_dic.keys(),
                        msg=msg + "attribute: [%s] not in gen_arr_info_dic.keys():\n%s" % (
                            info_k, repr(gen_arr_info_dic.keys())))
                    self.assertEqual(
                        info_v, gen_arr_info_dic[info_k],
                        msg=msg + "gen_arr_info_dic[%s]: '%s' != '%s'" % (
                            info_k, gen_arr_info_dic[info_k], info_v))


                    self.assertTrue(
                        hasattr(gen_arr_info, info_k),
                        msg=msg + "gen_arr_info of type [%s] does not have attribute: [%s]" % (
                            type(gen_arr_info), info_k))
                    self.assertEqual(
                        info_v, getattr(gen_arr_info, info_k),
                        msg=msg + "gen_arr_info[%s]: '%s' != '%s'" % (
                            info_k, getattr(gen_arr_info, info_k), info_v))


    def test_get_slot_class_and_field_information_single_level(self):
        from rqt_py_common.message_field_type_helpers import get_slot_class_and_field_information

        slot_class = self.val_slot_class
        slot_array_info = self.val_slot_array_info

        top_level_class = Val
        slot_values = list(slot_array_info.keys())

        for i, slot_value in enumerate(slot_values):
            target = "/" + slot_value
            target_class, field_info = get_slot_class_and_field_information(
                top_level_class, slot_value
            )

            field_info_dict = field_info.to_dict()

            base_msg = '[msg_class = Val, slot_value = "%s"]\t' % (slot_value)
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

    def test_get_slot_class_and_field_information_multi_level(self):
        from rqt_py_common.message_field_type_helpers import get_slot_class_and_field_information

        val_slot_class = self.val_slot_class
        val_slot_array_info = self.val_slot_array_info
        array_val_slot_class = self.array_val_slot_class
        array_val_slot_array_info = self.array_val_slot_array_info

        array_val_slots = list(array_val_slot_array_info.keys())
        array_val_slots_len = len(array_val_slots)
        for i in [i for i in range(array_val_slots_len) if array_val_slots[i]]:
            for j, val_value in [(j, v) for (j, v) in enumerate(val_slot_array_info.keys()) if v]:
                if val_value:
                    key = array_val_slots[i] + '/' + val_value
                    array_val_slot_array_info[key] = val_slot_array_info[val_value]
                    array_val_slot_class.append(val_slot_class[j])

        for i, slot_value in enumerate(array_val_slot_array_info.keys()):
            target = "/" + slot_value
            target_class, field_info = get_slot_class_and_field_information(
                ArrayVal, slot_value
            )

            field_info_dict = field_info.to_dict()

            base_msg = '[class = "ArrayVal", slot_value = "%s"]\t' % (slot_value)

            self.assertTrue(target_class is not None, msg=base_msg + '"%s" is None' % target_class)
            self.assertTrue(field_info is not None, msg=base_msg + '"%s" is None' % field_info)
            self.assertEqual(target_class, array_val_slot_class[i])
            for info_k, info_v in array_val_slot_array_info[slot_value].items():
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

    def test_get_field_python_class(self):  # noqa: D102
        from rqt_py_common.message_field_type_helpers import get_field_python_class
        from rqt_py_common.message_helpers import get_message_class

        field_type_to_python_type = {
            'boolean'                   : bool,
            'octet'                     : bytes,
            'float'                     : float,
            'double'                    : float,
            'uint8'                     : int,
            'int8'                      : int,
            'int16'                     : int,
            'int32'                     : int,
            'int64'                     : int,
            'uint8'                     : int,
            'uint16'                    : int,
            'uint32'                    : int,
            'uint64'                    : int,
            'string'                    : str,
            'rqt_py_common/ArrayVal'    : ArrayVal,
            'rqt_py_common/Val'         : Val,
            'string<5>'                 : str,
            'wstring<1>'                 : str,
            'int8[3]'                   : list,
            'sequence<int8>'            : list,
            'sequence<int8, 3>'         : list,
            'string[3]'                 : list,
            'sequence<string>'          : list,
            'string<5>[3]'              : list,
            'sequence<string<5>, 10>'   : list,
            'sequence<string<5>>'       : list,
            'sequence<string, 10>'      : list,
            # Some invalid
            'float[]'                   : None,
            'rqt_py_common/ArrayVal[]'  : None,
            'sequence<string,'          : None
        }

        for field_type, expected_python_type in field_type_to_python_type.items():
            self.assertEqual(
                get_field_python_class(field_type),
                expected_python_type,
                msg='get_field_python_class(\'%s\') != \'%s\'' % (
                    field_type, expected_python_type)
                )