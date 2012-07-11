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
from QtGui import QDialog, QDialogButtonBox
from QtCore import QDateTime, Signal
from qt_gui.qt_binding_helper import loadUi
from datetime import datetime

class TimeDialog(QDialog):
    """
    Dialog for returning a time range. To implement the ignore button a callback
    function in the caller must be connected to the ignore button. This way you
    can set a variable to check if the ignore button was clicked and behave
    accordingly. This functionality appears to be a bug with QT modal dialogs
    """
    ignore_button_clicked = Signal()
    def __init__(self):
        super(QDialog, self).__init__()
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'time_dialog.ui')
        loadUi(ui_file, self)
        def click_handler(button):
            if button == self.button_box.button(QDialogButtonBox.Ignore):
                self.ignore_button_clicked.emit()
                self.accept()
        self.button_box.clicked.connect(click_handler)
    
    def set_time(self, mintime=None, maxtime=None):
        if maxtime is None:
            time = datetime.now()
            self.min_dateedit.setDateTime(time)
            self.max_dateedit.setDateTime(time)
        else:
            dtime = QDateTime()
            dtime.setTime_t(int(mintime[:mintime.find('.')]))
            dtime = dtime.addMSecs(int(mintime[mintime.find('.')+1:mintime.find('.')+4]))
            self.min_dateedit.setDateTime(dtime)
            dtime = QDateTime()
            dtime.setTime_t(int(maxtime[:maxtime.find('.')]))
            dtime = dtime.addMSecs(int(maxtime[maxtime.find('.')+1:maxtime.find('.')+4]))
            self.max_dateedit.setDateTime(dtime)


