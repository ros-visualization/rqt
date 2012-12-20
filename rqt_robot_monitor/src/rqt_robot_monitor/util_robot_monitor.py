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
# Author: Isaac Saito, Ze'ev Klapow

import os
import copy

import rospy
from diagnostic_msgs.msg import DiagnosticStatus
from python_qt_binding.QtGui import QColor, QIcon

class Util(object):
    """
    
    """
    #TODO(Isaac) Utils and common configs are mixed in this class.
    
    _SECONDS_TIMELINE = 30
    
    # Instantiating icons that show the device status.
    _ERR_ICON = QIcon.fromTheme('dialog-error')  # 'face-angry')
    _WARN_ICON = QIcon.fromTheme('dialog-warning')  # 'face-sick')
    _OK_ICON = QIcon.fromTheme('emblem-default')  # 'face-laugh')
    # Added following this QA thread http://goo.gl/83tVZ  
    _STALE_ICON = QIcon.fromTheme('dialog-question')  # 'face-tired')
      
    _IMG_DICT = {0: _OK_ICON, 1: _WARN_ICON, 2: _ERR_ICON, 3: _STALE_ICON}
    
    _COLOR_DICT = {0: QColor(85, 178, 76),
                   1: QColor(222, 213, 17),
                   2: QColor(178, 23, 46),
                   3: QColor(40, 23, 176)
                   }
    # DiagnosticStatus dosn't have Stale status. Related QA:http://goo.gl/83tVZ
    # It's not ideal to add STALE to DiagnosticStatus as you see in that thread,
    # but here this addition is only temporary for the purpose of implementation
    # simplicity.  
    DiagnosticStatus.STALE = 3
    
    _DICTKEY_TIMES_ERROR = 'times_errors'
    _DICTKEY_TIMES_WARN = 'times_warnings'
    _DICTKEY_INDEX = 'index'
    _DICTKEY_STATITEM = 'statitem'

    
    def __init__(self):
        super(Util, self).__init__()
    
    @staticmethod
    def _update_status_images(diagnostic_status, statusitem):
        """
        Taken from robot_monitor.robot_monitor_panel.py.
        
        :type status: DiagnosticStatus         
        :type node: StatusItem 
        :author: Isaac Saito 
        """

        name = diagnostic_status.name
        if (name is not None):
            # level = diagnosis_status.level
            level = diagnostic_status.level                
            rospy.logdebug('New diagnostic_status level: %s. Last lv: %s name: %s',
                       level, statusitem.last_level, name)        
            if (diagnostic_status.level != statusitem.last_level):  
                # TODO Apparently diagnosis_status doesn't contain last_level. 
                statusitem.setIcon(0, Util._IMG_DICT[level])
                statusitem.last_level = level
                return
               
    @staticmethod
    def get_nice_name(status_name):
        """
        :param: status_name is a string that may consists of status names that 
                  are delimited by slash.
        :rtype: str
        """
        name = status_name.split('/')[-1]
        rospy.logdebug(' get_nice_name name = %s', name)
        return name
    
    @staticmethod
    def remove_parent_name(status_name):
        return ('/'.join(status_name.split('/')[2:])).strip()
    
    @staticmethod
    def get_parent_name(status_name):
        return ('/'.join(status_name.split('/')[:-1])).strip()
    
    @staticmethod
    def gen_headline_status_green(diagnostic_status):
        # return "%s : %s" % (get_nice_name(diagnostic_status.status.name), 
        #                                   diagnostic_status.status.message)
        return "%s" % Util.get_nice_name(diagnostic_status.name)
    
    @staticmethod
    def gen_headline_warn_or_err(diagnostic_status):
        return "%s : %s" % (Util.get_nice_name(diagnostic_status.name),
                            diagnostic_status.message)

    @staticmethod
    def _get_color_for_message(msg, mode = 0):
        """ 
        
        Copied from robot_monitor.

        :param msg: Either DiagnosticArray or DiagnosticsStatus.
        :param mode: int. When 0, this func will iterate msg to find 
                     DiagnosticsStatus.level and put it into a dict.
                     When 1, this func finds DiagnosticsStatus.level from msg
                     without iteration (thus, msg is expected to be
                     DiagnosticsStatus instance). 
        """
                
        level = 0
        min_level = 255
        
        lookup = {}
        for status in msg.status:
            lookup[status.name] = status
            
        names = [status.name for status in msg.status]
        names = [name for name in names if len(Util.get_parent_name(name)) == 0]
        for name in names:
            status = lookup[name]
            if (status.level > level):
                level = status.level
            if (status.level < min_level):
                min_level = status.level

        # Stale items should be reported as errors unless all stale
        if (level > 2 and min_level <= 2):
            level = 2
                
        #return Util._IMG_DICT[level]
        rospy.logdebug(' get_color_for_message color lv=%d', level)
        return Util._COLOR_DICT[level]
    
    @staticmethod
    def get_correspondent(key, list_statitem):
        """
        
        :type key: String.  
        :type list_statitem: DiagnosticsStatus
        :rtype: StatusItem
        """
        names_from_list = [Util.get_nice_name(k.name) for k in list_statitem]
        key_niced = Util.get_nice_name(key)
        index_key = -1        
        statitem_key = None
        if key_niced in names_from_list:
            index_key = names_from_list.index(key_niced)
            statitem_key = list_statitem[index_key]
            rospy.logdebug(' get_correspondent index_key=%s statitem_key=%s',
                          index_key, statitem_key)
        return {Util._DICTKEY_INDEX : index_key,
                Util._DICTKEY_STATITEM : statitem_key}

    @staticmethod
    def get_children(name, diag_array):
        """
        
        :type msg: DiagnosticArray
        :rtype: DiagnosticStatus[]
        """
        
        ret = []
        for k in diag_array.status:  # k is DiagnosticStatus. 
            if k.name.startswith(name):  # Starting with self.name means k 
                                       # is either top/parent node or its child.
                if not k.name == name:  # Child's name must be different 
                                            # from that of the top/parent node.  
                    ret.append(k)
        return ret      
         