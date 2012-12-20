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

from collections import deque
from math import floor
import os

from python_qt_binding import loadUi
from python_qt_binding.QtCore import QPointF, QSize, Qt, Signal
from python_qt_binding.QtGui import QColor, QIcon, QGraphicsScene, QWidget
import rospy
import rospkg

from .timeline import TimelineView
from .util_robot_monitor import Util

class TimelinePane(QWidget):
    """
    This class defines the pane where timeline and its related components
    are displayed. 
    """
    
    sig_update = Signal()
        
    def __init__(self, parent, len_timeline,
                 color_callback,
                 pause_callback=None):
        """
        
        :param color_callback: Not directly used within this class. Instead,
        this will be passed and used in TimelineView class.
        """
        #TODO(Isaac) pause_callback is MUST since it's used in both by
        #            RobotMonitorWidget & InspectorWindow. 
        
        super(TimelinePane, self).__init__()
        self._parent = parent
        
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_robot_monitor'), 'resource', 'rqt_robot_monitor_timelinepane.ui')
        loadUi(ui_file, self, {'TimelineView': TimelineView})   
        
        self._pause_callback = pause_callback
        self._timeline_view.set_init_data(1, len_timeline, 5, color_callback)
  
        self._scene = QGraphicsScene(self._timeline_view)
        self._timeline_view.setScene(self._scene)        
        self._timeline_view.show()
        
        self._queue_diagnostic = deque()
        self._len_timeline = len_timeline
        self._paused = False
        self._tracking_latest = True
        self._last_sec_marker_at = 2
        self._last_msg = None
        
        self._pause_button.clicked[bool].connect(self._pause)
        self.sig_update.connect(self._timeline_view.slot_redraw)     
     
    def mouse_release(self, event):
        """
        :type event: QMouseEvent 
        """
        xpos_clicked = event.x()
        width_each_cell_shown = float(
                       self._timeline_view.viewport().width()) / len(
                                                   self._queue_diagnostic)
        i = int(floor(xpos_clicked / width_each_cell_shown))
        rospy.logdebug('mouse_release i=%d width_each_cell_shown=%s', 
                       i, width_each_cell_shown)        

        msg = self._queue_diagnostic[i]
        if msg:
            self._parent.on_pause(True, msg)
            
            if not self._pause_button.isChecked():
                self._pause_button.toggle()

    def get_worst(self, msg):
        lvl = 0
        for status in msg.status:
            if status.level > lvl:
                lvl = status.level
        return lvl

    def _pause(self, paused):
        """
        Should be the only interface for pausing timeline pane and timeline 
        itself (which is not as of now..).
        
        :type paused: bool
        """
        #TODO(Isaac) Think about optimal way to call functions in parent class
        # (pause, _unpause). Use callback or just calling via parent instance?
        
        rospy.logdebug('TimelinePane pause isPaused?=%s', paused)
        if paused:
            self._pause_button.setDown(True)
            self._paused = True
            self._tracking_latest = False
            self._parent.pause(self._queue_diagnostic[-1])
        else:
            self._pause_button.setDown(False)
            self._paused = False
            self._tracking_latest = True
            self._parent.unpause(self._queue_diagnostic[-1])
            rospy.logdebug('Objs; parent=%s, parent._unpause obj=%s', 
                           self._parent,
                           self._parent.unpause)

    def on_slider_scroll(self, evt):
        """
        
        :type evt: QMouseEvent 
        """
        
        xpos_marker = self._timeline_view.get_xpos_marker() - 1
        rospy.logdebug('on_slider_scroll xpos_marker=%s _last_sec_marker_at=%s',
                      xpos_marker, self._last_sec_marker_at)
        if (xpos_marker == self._last_sec_marker_at):
            # Clicked the same pos as last time.
            return
        elif (xpos_marker >= len(self._queue_diagnostic)):
            # When clicked out-of-region
            return

        self._last_sec_marker_at = xpos_marker
        
        self._pause(True)
        
        # Fetch corresponding previous DiagsnoticArray instance from queue,
        # and sig_update trees.
        msg = self._queue_diagnostic[xpos_marker]
        self._parent.new_diag(msg, True)

    def new_diagnostic(self, msg):
        """
        Callback for new msg for TimelinePane class.
        
        Puts new msg into a queue, update the length of timeline. Also emits
        a signal to notify another callbacks.        
        This ignores new msg if timeline is paused.
                
        :type msg: Either DiagnosticArray or DiagnosticsStatus. Can be 
                   determined by __init__'s arg "msg_callback".
        """
        
        self._last_msg = msg # Shouldn't this sit after self._paused return?
                             # Original robot_monitor does this way.   
        
        # if (self._message_receipt_callback is not None):
        #    self._message_receipt_callback(msg)
        # # This is done here in robot_monitor. 
        # # In rqt_robot_monitor, however, it's done in RobotMonitorWidget.
        
        if (self._paused):
            return
        
        self._queue_diagnostic.append(msg)
        if (len(self._queue_diagnostic) > self._len_timeline):
            self._queue_diagnostic.popleft()
            
        new_len = len(self._queue_diagnostic)
        self._timeline_view.set_range(1, new_len)
        
        self.sig_update.emit()   
        rospy.logdebug(' TimelinePane new_diagnostic new_len=%d', new_len)
           
    def redraw(self):
        self.sig_update.emit() 
          
    def get_diagnostic_queue(self):
        """
        
        :return: a queue that contains either DiagnosticArray or 
                 DiagnosticsStatus. Depends on the parent class - 
                 if RobotMonitorWidget is the parent, the former type is returned.
                 if InspectorWidget the latter.
        """
        return self._queue_diagnostic
