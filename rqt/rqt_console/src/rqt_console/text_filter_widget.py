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

class TextFilterWidget(QWidget):
    def __init__(self, parentfilter, display_list_args):
        super(TextFilterWidget, self).__init__()
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'text_filter_widget.ui')
        loadUi(ui_file, self)
        self.setObjectName('TextFilterWidget')
        self._parentfilter = parentfilter  # When data is changed we need to store it in the parent filter
        
        self.text_edit.textChanged.connect(self.handle_text_changed)
        self.regex_check_box.clicked[bool].connect(self.handle_regex_clicked)

        self.handle_text_changed()
    
    def set_text(self, text):
        self.text_edit.setText(text)

    def set_regex(self, checked):
        self.regex_check_box.setChecked(checked)
        self.handle_regex_clicked(checked)

    def handle_text_changed(self):
        self._parentfilter.set_text(self.text_edit.text())

    def handle_regex_clicked(self, clicked):
        self._parentfilter.set_regex(clicked)

    def repopulate(self):
        pass

    def save_settings(self, settings):
        settings.set_value('text', self._parentfilter._text)
        settings.set_value('regex', self._parentfilter._regex)
        return

    def restore_settings(self, settings):
        text = settings.value('text')
        self.set_text(text)
        self.handle_text_changed()

        regex = settings.value('regex') in [True, 'true']
        self.set_regex(regex)
        self.handle_regex_clicked(regex)
        return
