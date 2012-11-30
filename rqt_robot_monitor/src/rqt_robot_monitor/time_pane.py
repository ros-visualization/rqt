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
from math import floor

import roslib;roslib.load_manifest('rqt_robot_monitor')
import rospy
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget, QGraphicsScene, QColor
from python_qt_binding.QtCore import Signal, Qt

from timeline import TimelineView

class TimelinePane(QWidget):
    update = Signal()
    def __init__(self, parent):
        super(TimelinePane, self).__init__()
        self.parent = parent
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 
                               'rqt_robot_monitor_timelinepane.ui')
        loadUi(ui_file, self, {'TimelineView': TimelineView})
  
        self._scene = QGraphicsScene(self._timeline_view)
        self._colors = [QColor('green'), QColor('yellow'), QColor('red')]
        self._timeline_view.setScene(self._scene)
        
        self._messages = [None for x in range(20)] # DiagnosticStatus
        self._mq = [1 for x in range(20)] 

        self.pause_button.clicked.connect(self._pause)
        self.update.connect(self.redraw)

    def redraw(self):
        rospy.logdebug('in TimelinePane redraw 11')
        self._scene.clear()
        #self._scene 
        rospy.logdebug('TimelinePane redraw BEFORE LOOP')
        for i, m in enumerate(self._mq):
            w = float(self._timeline_view.viewport().width()) / len(self._mq)
            h = self._timeline_view.viewport().height()
            rect = self._scene.addRect(
                                       w * i,
                                       0,
                                       w,
                                       h,
                                       QColor('black'),
                                       self._colors[m])
            rospy.logdebug('in TimelinePane redraw i=%d m=%d', i, m)
            
        rospy.logdebug('TimelinePane redraw ENDS')

    '''
    @param event: QMouseEvent 
    '''
    def mouse_release(self, event):
        xpos_clicked = event.x()
        width_each_cell_shown = float(
                       self._timeline_view.viewport().width()) / len(self._mq)
        i = int(floor(xpos_clicked / width_each_cell_shown))

        msg = self._messages[i]
        if msg:
            self._pause(msg)
            if not self.pause_button.isChecked():
                self.pause_button.toggle()

    def resizeEvent(self, event):
        self.redraw()

    def get_worst(self, msg):
        lvl = 0
        for status in msg.status:
            if status.level > lvl:
                lvl = status.level

        return lvl

    '''
    @param msg: DiagnosticStatus 
    '''
    def add_message(self, msg):
        rospy.logdebug('\tTimelinePane add_message 1')
        self._messages = self._messages[1:]
        self._messages.append(msg)

        self._mq = self._mq[1:]
        try:
            lvl = msg.level            
        except AttributeError:
            lvl = self.get_worst(msg)

        rospy.logdebug('\tTimelinePane add_message lvl=%s', lvl)
        if lvl > 2:
            lvl = 2

        self._mq.append(lvl)

        self.update.emit()
        rospy.logdebug('\tTimelinePane add_message AFTER EMIT')

    '''
    @param state:  
    '''
    def _pause(self, state):
        if state:
            self.parent.pause(self._messages[-1])
        else:
            self.parent.unpause()
