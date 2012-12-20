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

import rospy

from python_qt_binding.QtCore import Qt, Signal
from python_qt_binding.QtGui import QPushButton, QTextEdit, QVBoxLayout, QWidget

from .abst_status_widget import AbstractStatusWidget
from .status_snapshot import StatusSnapshot
from .time_pane import TimelinePane
from .util_robot_monitor import Util

class InspectorWindow(AbstractStatusWidget):
    _sig_write = Signal(str, str)
    _sig_newline = Signal()
    _sig_close_window = Signal()
    _sig_clear = Signal()
    
    def __init__(self, status, close_callback):
        """      
               
        :type status: DiagnosticStatus
        :param close_callback: When the instance of this class (InspectorWindow)
                               terminates, this callback gets called.
        """
        #TODO(Isaac) UI construction that currently is done in this method,
        #            needs to be done in .ui file.
               
        super(InspectorWindow, self).__init__()
        self.status = status
        self._close_callback = close_callback
        self.setWindowTitle(status.name)
        self.paused = False

        self.layout_vertical = QVBoxLayout(self)
        
        self.disp = QTextEdit(self)
        self.snapshot = QPushButton("StatusSnapshot")

        self.timeline_pane = TimelinePane(self, 
                                          Util._SECONDS_TIMELINE,
                                          self.get_color_for_value)

        self.layout_vertical.addWidget(self.disp, 1)
        self.layout_vertical.addWidget(self.timeline_pane, 0)
        self.layout_vertical.addWidget(self.snapshot)

        self.snaps = []
        self.snapshot.clicked.connect(self._take_snapshot)

        self._sig_write.connect(self._write_key_val)
        self._sig_newline.connect(lambda: self.disp.insertPlainText('\n'))
        self._sig_clear.connect(lambda: self.disp.clear())
        self._sig_close_window.connect(self._close_callback)

        self.setLayout(self.layout_vertical)
        self.setGeometry(0, 0, 400, 600)  # TODO better to be configurable where to appear.
        self.show()
        self.update_status_display(status)
        
    def closeEvent(self, event):    
        # emit signal that should be slotted by StatusItem
        self._sig_close_window.emit()        
        self.close()
                
    def _write_key_val(self, k, v):
        self.disp.setFontWeight(75)
        self.disp.insertPlainText(k)
        self.disp.insertPlainText(': ')

        self.disp.setFontWeight(50)
        self.disp.insertPlainText(v)
        self.disp.insertPlainText('\n')

    def pause(self, msg):
        rospy.logdebug('InspectorWin pause PAUSED')
        self.paused = True
        self.update_status_display(msg);

    def unpause(self, msg):
        rospy.logdebug('InspectorWin pause UN-PAUSED')
        self.paused = False

    def new_diag(self, msg, is_forced = False):
        """
        Overridden from AbstractStatusWidget 
        
        :type status: DiagnosticsStatus
        """
        
        if not self.paused:
            self.update_status_display(msg)
            rospy.logdebug('InspectorWin _cb len of queue=%d self.paused=%s',
                          len(self.timeline_pane._queue_diagnostic), self.paused)
        else:
            if is_forced:
                self.update_status_display(msg, True)
                rospy.logdebug('@@@InspectorWin _cb PAUSED window updated')
            else:
                rospy.logdebug('@@@InspectorWin _cb PAUSED not updated')
                   
    def update_status_display(self, status, is_forced = False):
        """
        :type status: DiagnosticsStatus 
        """
        
        if not self.paused or (self.paused and is_forced):
            self.timeline_pane.new_diagnostic(status)
              
            rospy.logdebug('InspectorWin update_status_display 1')

            self.status = status
            self._sig_clear.emit()
            self._sig_write.emit("Full Name", status.name)
            self._sig_write.emit("Component", status.name.split('/')[-1])
            self._sig_write.emit("Hardware ID", status.hardware_id)
            self._sig_write.emit("Level", str(status.level))
            self._sig_write.emit("Message", status.message)
            self._sig_newline.emit()

            for v in status.values:
                self._sig_write.emit(v.key, v.value)

    def _take_snapshot(self):
        snap = StatusSnapshot(self.status)
        self.snaps.append(snap)
        
    def get_color_for_value(self, queue_diagnostic, color_index):
        """
        Overridden from AbstractStatusWidget.
        
        :type color_index: int
        """
                
        rospy.logdebug('InspectorWindow get_color_for_value ' +
                       'queue_diagnostic=%d, color_index=%d',
                       len(queue_diagnostic), color_index)
        lv_index = queue_diagnostic[color_index - 1].level
        return Util._COLOR_DICT[lv_index]        
        