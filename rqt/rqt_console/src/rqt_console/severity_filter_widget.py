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
from QtCore import Qt
from qt_gui.qt_binding_helper import loadUi


class SeverityFilterWidget(QWidget):
    def __init__(self, parentfilter, severities):
        super(SeverityFilterWidget, self).__init__()
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'severity_filter_widget.ui')
        loadUi(ui_file, self)
        self.setObjectName('SeverityFilterWidget')
        self._parentfilter = parentfilter  # When data is changed we need to store it in the parent filter
        self.debug_checkbox.stateChanged[int].connect(self.debug_callback)
        self.info_checkbox.stateChanged[int].connect(self.info_callback)
        self.warning_checkbox.stateChanged[int].connect(self.warning_callback)
        self.error_checkbox.stateChanged[int].connect(self.error_callback)
        self.fatal_checkbox.stateChanged[int].connect(self.fatal_callback)
        self._severity_list = severities

    def debug_callback(self, checked):
        self._parentfilter.set_debug(checked == Qt.Checked)
    
    def info_callback(self, checked):
        self._parentfilter.set_info(checked == Qt.Checked)
    
    def warning_callback(self, checked):
        self._parentfilter.set_warning(checked == Qt.Checked)
    
    def error_callback(self, checked):
        self._parentfilter.set_error(checked == Qt.Checked)
    
    def fatal_callback(self, checked):
        self._parentfilter.set_fatal(checked == Qt.Checked)

    def repopulate(self):
        # filters need this slot but it is not used here since it doesn't make
        # sense to dynamically add Severity levels
        pass
