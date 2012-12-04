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

import roslib;roslib.load_manifest('rqt_robot_monitor')
import rospy

from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget, QGraphicsView, QColor, QIcon
from python_qt_binding.QtCore import Signal, Qt

class TimelineView(QGraphicsView):
    #def __init__(self, parent, min_val, max_val, val, color_callback):
    def __init__(self, parent):    
        super(TimelineView, self).__init__()
        self.parent = parent
        
        self._timeline_marker = QIcon.fromTheme('system-search')
        
        #self._min = min_val
        #self._max = max_val
        #self._val = val
        #self._color_callback = color_callback

#    def resizeEvent(self, event):
#        self.adjustSize() #TODO Make sure this meets my requirement.
        
    '''
    @param event: QMouseEvent 
    '''
#    def mouseReleaseEvent(self, event):
#        self.parent.mouse_release(event)
#        rospy.logdebug('\tTimelineView mouseReleaseEvent ****')
        
#    def on_size(self, event):
#        self.Refresh()

