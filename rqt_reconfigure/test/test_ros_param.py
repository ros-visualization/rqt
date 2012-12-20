# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
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
#  * Neither the name of Willow Garage, Inc. nor the names of its
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
#
# Author: Isaac Saito

import unittest

from rqt_reconfigure.ros_param import Parameter

'''
@author: Isaac Saito
'''
class MyTest(unittest.TestCase):
    
    # All following vars need to be modified accordingly whenever you give 
    # different raw param name. 
    # <here>
    _param_name_raw = '/param_top/sub/subsub'
    _param_name_toplv = 'param_top'
    _len_param_name = 3
    # </here>
            
    def setUp(self):
        unittest.TestCase.setUp(self)
        
        self._param = Parameter(self._param_name_raw)
 
    def tearDown(self):
        unittest.TestCase.tearDown(self)
        del self._param
                
    def test_get_param_name_toplv(self):
        self.assertEqual(self._param.get_param_name_toplv(), 
                         self._param_name_toplv)
        
    def test_get_param_names(self):
        self.assertEqual(len(self._param.get_param_names()), 
                         self._len_param_name - 1)

if __name__ == '__main__':
    unittest.main()