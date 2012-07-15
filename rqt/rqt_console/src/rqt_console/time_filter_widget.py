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

import os

from QtGui import QIcon, QWidget
from QtCore import QDateTime, Qt
from qt_gui.qt_binding_helper import loadUi
from datetime import datetime

class TimeFilterWidget(QWidget):
    def __init__(self, parentfilter, display_list_args):
        super(TimeFilterWidget, self).__init__()
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'time_filter_widget.ui')
        loadUi(ui_file, self)
        self.setObjectName('TimeFilterWidget')
        self._parentfilter = parentfilter  # When data is changed we need to store it in the parent filter
        
        mintime, maxtime = display_list_args[0]()
        if mintime != -1:
            mintime = str(mintime).split('.')
            maxtime = str(maxtime).split('.')
            
            time = QDateTime()
            time.setTime_t(int(mintime[0]))
            mintime = time.addMSecs(int(str(mintime[1]).zfill(9)[:3]))
            self.start_datetime.setDateTime(mintime)
            time.setTime_t(int(maxtime[0]))
            maxtime = time.addMSecs(int(str(maxtime[1]).zfill(9)[:3]))
            self.stop_datetime.setDateTime(maxtime)
        else:
            self.start_datetime.setDateTime(datetime.now())
            self.stop_datetime.setDateTime(datetime.now())
            
        self.start_datetime.dateTimeChanged[QDateTime].connect(self.handle_start_changed)
        self.stop_datetime.dateTimeChanged[QDateTime].connect(self.handle_stop_changed)
        self.stop_enabled_check_box.clicked[bool].connect(self.handle_stop_enabled_changed)

    def handle_start_changed(self, datetime):
        self._parentfilter.set_start_time(datetime)

    def handle_stop_changed(self, datetime):
        self._parentfilter.set_stop_time(datetime)

    def handle_stop_enabled_changed(self, checked):
        self._parentfilter.set_stop_time_enabled(checked)

    def repopulate(self):
        pass
