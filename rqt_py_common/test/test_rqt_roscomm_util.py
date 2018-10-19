#!/usr/bin/python

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
# Author: Isaac Saito

import unittest

from rqt_py_common.rqt_roscomm_util import RqtRoscommUtil


class TestRqtRoscommUtil(unittest.TestCase):

    """
    @author: Isaac Saito
    """

    def setUp(self):
        unittest.TestCase.setUp(self)

    def tearDown(self):
        unittest.TestCase.tearDown(self)
        # del self._model

    def test_iterate_packages(self):
        """
        Not a very good test because it only tests that the number of packages is non-zero
        """
        pkg_num_sum = 0
        for pkg in RqtRoscommUtil.iterate_packages('launch'):
            pkg_num_sum += 1
            print('pkg={}'.format(pkg))

        print(pkg_num_sum)
        self.assertNotEqual(pkg_num_sum, 0)

    def test_list_files(self):
        """
        Not a very good test because it tests that there is at least one .msg file in std_msg
        """
        file_num = 0
        pkg_name = 'std_msgs'
        subdir = 'msg'
        file_ext = '.msg'
        files = RqtRoscommUtil.list_files(pkg_name, subdir, file_ext)
        for file in files:
            file_num += 1
            print('file={}'.format(file))

        print(file_num)
        self.assertNotEqual(file_num, 0)


if __name__ == '__main__':
    unittest.main()
