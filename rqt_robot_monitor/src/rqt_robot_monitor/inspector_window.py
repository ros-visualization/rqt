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

import roslib;roslib.load_manifest('rqt_robot_monitor')
import rospy

from python_qt_binding.QtGui import QWidget, QVBoxLayout, QTextEdit, QPushButton
from python_qt_binding.QtCore import Signal, Qt

from timeline_widget import TimelineWidget

class InspectorWindow(QWidget):
    write = Signal(str, str)
    newline = Signal()
    _close_window = Signal()
    clear = Signal()
    
    def __init__(self, status):
        super(InspectorWindow, self).__init__()
        self.status = status
        self.setWindowTitle(status.name)
        self.paused = False

        layout_vertical = QVBoxLayout(self)
        
        self.disp = QTextEdit(self)
        self.snapshot = QPushButton("Snapshot")

        self.timeline = TimelineWidget(self)

        layout_vertical.addWidget(self.disp, 1)
        layout_vertical.addWidget(self.timeline, 0)
        layout_vertical.addWidget(self.snapshot)

        self.snaps = []
        self.snapshot.clicked.connect(self.take_snapshot)

        self.write.connect(self.write_kv)
        self.newline.connect(lambda: self.disp.insertPlainText('\n'))
        self.clear.connect(lambda: self.disp.clear())

        self.setLayout(layout_vertical)
        self.setGeometry(0, 0, 300, 400)  # TODO better to be configurable where to appear. 
        self.show()
        self.update_children(status)
    '''
    Delegated from super class.
    @author: Isaac Saito
    '''
    def closeEvent(self, event):    
        # emit signal that should be slotted by StatusItem
        self._close_window.emit()        
        self.close()
                
    def write_kv(self, k, v):
        self.disp.setFontWeight(75)
        self.disp.insertPlainText(k)
        self.disp.insertPlainText(': ')

        self.disp.setFontWeight(50)
        self.disp.insertPlainText(v)
        self.disp.insertPlainText('\n')

    def pause(self, msg):
        self.update_children(msg);
        self.paused = True

    def unpause(self):
        self.paused = False

    def update_children(self, status):
        if not self.paused:
            self.status = status
            self.timeline.add_message(status)

            self.clear.emit()
            self.write.emit("Full Name", status.name)
            self.write.emit("Component", status.name.split('/')[-1])
            self.write.emit("Hardware ID", status.hardware_id)
            self.write.emit("Level", str(status.level))
            self.write.emit("Message", status.message)
            self.newline.emit()

            for v in status.values:
                self.write.emit(v.key, v.value)

    def take_snapshot(self):
        snap = Snapshot(self.status)
        self.snaps.append(snap)
        
class Snapshot(QTextEdit):
    """Display a single static status message. Helps facilitate copy/paste"""
    def __init__(self, status):
        super(Snapshot, self).__init__()

        self.write("Full Name", status.name)
        self.write("Component", status.name.split('/')[-1])
        self.write("Hardware ID", status.hardware_id)
        self.write("Level", status.level)
        self.write("Message", status.message)
        self.insertPlainText('\n')

        for value in status.values:
            self.write(value.key, value.value)

        self.setGeometry(0, 0, 300, 400)
        self.show()

    def write(self, k, v):
        self.setFontWeight(75)
        self.insertPlainText(str(k))
        self.insertPlainText(': ')
     
        self.setFontWeight(50)
        self.insertPlainText(str(v))
        self.insertPlainText('\n')           
