#!/usr/bin/python3

# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# * Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above
# copyright notice, this list of conditions and the following
# disclaimer in the documentation and/or other materials provided
# with the distribution.
# * Neither the name of Willow Garage, Inc. nor the names of its
# contributors may be used to endorse or promote products derived
# from this software without specific prior written permission.
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
#
# Author: Isaac Saito, Michael Lautman

import os
import unittest


class TestRqtRoscommUtil(unittest.TestCase):
    """@author: Isaac Saito"""

    def setUp(self):
        unittest.TestCase.setUp(self)

    def tearDown(self):
        unittest.TestCase.tearDown(self)
        # del self._model

    # TODO(mlautman): Replace this test with an appropriate ROS2 test.
    def test_iterate_packages(self):
        """Not a very good test."""
        # Note: mlautman 11/2/18
        #       This test has been removed since the iterate_package method in RqtRoscommUtil
        #       is deprecated in ROS2
        from rqt_py_common.rqt_roscomm_util import RqtRoscommUtil

        pkg_num_sum = 0
        for pkg, msg_dir_path in RqtRoscommUtil.iterate_packages('msg'):
            pkg_num_sum += 1
            print('pkg:\t{}\t msg dir path:\t{}'.format(pkg, msg_dir_path))
            path_to_msg_dir, msg_dir = os.path.split(msg_dir_path)
            self.assertEqual(msg_dir, 'msg')

        print('number of packages:\t{}'.format(pkg_num_sum))
        self.assertNotEqual(pkg_num_sum, 0)
