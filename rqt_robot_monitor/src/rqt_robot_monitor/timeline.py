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
    def __init__(self, parent, min_val, max_val, val, color_callback):
    #def __init__(self, parent):
        super(TimelineView, self).__init__()
        self.parent = parent
        
        self._timeline_marker = QIcon.fromTheme('system-search')
        
        self._min = min_val
        self._max = max_val
        self._val = val
        self._color_callback = color_callback

    '''
    Override.
    '''        
    def paintEvent(self, event):
        dc = wx.PaintDC(self)
        dc.Clear()
        
        is_enabled = self.IsEnabled()

        (width, height) = self.GetSizeTuple()
        length = self._max + 1 - self._min
        value_size = width / float(length)
        for i in xrange(0, length):
            if (is_enabled):
                color = self._color_callback(i + self._min)
            else:
                color = wx.LIGHT_GREY
            end_color = wx.Colour(0.6 * color.Red(), 0.6 * color.Green(), 0.6 * color.Blue())
            dc.SetPen(wx.Pen(color))
            dc.SetBrush(wx.Brush(color))
            start = i * value_size
            end = (i + 1) * value_size
            dc.GradientFillLinear(wx.Rect(start, 0, end, height), color, end_color, wx.SOUTH)
            
            if (i > 0):
                dc.SetPen(wx.BLACK_PEN)
                dc.DrawLine(start, 0, start, height)
            
        marker_x = ((self._val - 1) * value_size + 
                    (value_size / 2.0) - 
                    (self._timeline_marker_bitmap.GetWidth() / 2.0))
        dc.DrawBitmap(self._timeline_marker_bitmap, marker_x, 0, True)
        
    def resizeEvent(self, event):
        self.adjustSize() #TODO Make sure this meets my requirement.
        
    '''
    @param event: QMouseEvent 
    '''
    def mouseReleaseEvent(self, event):
        self.parent.mouse_release(event)
        rospy.logdebug('\tTimelineView mouseReleaseEvent ****')
        
    def on_size(self, event):
        self.Refresh()

 
    def on_mouse(self, event):
        if (event.LeftDown()):
            self._left_down = True
            rospy.loginfo('on_mouse 1')
        elif (event.LeftUp()):
            self._left_down = False
            self._set_val_from_x(event.GetX())
            wx.PostEvent(self.GetEventHandler(), 
                         wx.PyCommandEvent(wx.EVT_COMMAND_SCROLL_CHANGED.typeId, 
                                           self.GetId()))
            rospy.loginfo('on_mouse 2')
        
        if (self._left_down):
            self._set_val_from_x(event.GetX())
            wx.PostEvent(self.GetEventHandler(), 
                         wx.PyCommandEvent(wx.EVT_COMMAND_SCROLL_THUMBTRACK.typeId, 
                                           self.GetId()))
            rospy.loginfo('on_mouse 3')       
