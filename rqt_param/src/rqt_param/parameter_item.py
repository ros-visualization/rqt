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

from __future__ import division

import dynamic_reconfigure.client
from python_qt_binding.QtCore import Qt, QVariant
import rospy
from rqt_py_common.data_items import ReadonlyItem

class ParameterItem(ReadonlyItem):
    """
    IMPORTANT: set_param_name method must be called right after 
    the constructor is called.
    """
        
    def __init__(self, *args):
        """
        :param param_name: A string formatted as GRN (Graph Resource Names, see  
                           http://www.ros.org/wiki/Names). 
                           Example: /paramname/subpara/subsubpara/...
        """
        
        super(ParameterItem, self).__init__(*args)
    
    def set_param_name(self, param_name):
        
        self._param_name_raw = param_name
        
        #  separate param_name by forward slash
        self._list_paramname = param_name.split('/')
        
        #  Deleting the 1st elem which is zero-length str.
        del self._list_paramname[0]  
        
        self._nodename = self._list_paramname[0]
        
        rospy.logdebug('ParameterItem.__init__ param_name=%s  self._list_paramname[-1]=%s',
                       param_name,
                       self._list_paramname[-1])
                        
    def get_param_name_toplv(self):
        """
        :rtype: String of the top level param name.
        """ 

        return self._name_top
    
    def get_raw_param_name(self):
        return self._param_name_raw
    
    def get_param_names(self):
        """
        :rtype: List of string. Null if param
        """
    
        #TODO what if self._list_paramname is empty or null?
        return self._list_paramname

    def get_node_name(self):
        return self._nodename
        
    def type(self):
        return QStandardItem.UserType
