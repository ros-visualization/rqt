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

class CustomFilterWidget(QWidget):
    def __init__(self, parentfilter, display_list_args):
        super(CustomFilterWidget, self).__init__()
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'custom_filter_widget.ui')
        loadUi(ui_file, self)
        self.setObjectName('CustomFilterWidget')
        self._parentfilter = parentfilter  # When data is changed we need to store it in the parent filter

        # Text Filter Initialization
        self.text_edit.textChanged.connect(self.handle_text_changed)
        self.regex_check_box.clicked[bool].connect(self.handle_regex_clicked)

        self.handle_text_changed()
        
        # Severity Filter Initialization
        self.severity_list.itemSelectionChanged.connect(self.handle_severity_item_changed)
        newlist =  display_list_args[0]()
        for item in newlist:
            self.severity_list.addItem(item)
        
        # Topic Filter Initialization
        self._topic_list_populate_function = display_list_args[3]
        self._topic_function_argument = False
        self._topic_function_argument = display_list_args[4]
        self.topic_list.itemSelectionChanged.connect(self.handle_topic_item_changed)
        self._topic_display_list = []

        # Node Filter Initialization
        self._node_list_populate_function = display_list_args[1]
        self._node_function_argument = False
        self._node_function_argument = display_list_args[2]
        self.node_list.itemSelectionChanged.connect(self.handle_node_item_changed)
        self._node_display_list = []

        self.repopulate()

    def handle_node_item_changed(self):
        self._parentfilter._node.set_list(self.node_list.selectedItems())

    def handle_topic_item_changed(self):
        self._parentfilter._topic.set_list(self.topic_list.selectedItems())

    def handle_severity_item_changed(self):
        self._parentfilter._severity.set_list(self.severity_list.selectedItems())

    def handle_text_changed(self):
        self._parentfilter._message.set_text(self.text_edit.text())

    def handle_regex_clicked(self, clicked):
        self._parentfilter._message.set_regex(clicked)

    def repopulate(self):
        newlist =  self._topic_list_populate_function(self._topic_function_argument)
        if len(newlist) != len(self._topic_display_list):
            for item in newlist:
                if not item in self._topic_display_list:
                    self.topic_list.addItem(item)
        self._topic_display_list = list(set(newlist + self._topic_display_list))

        newlist =  self._node_list_populate_function(self._node_function_argument)
        if len(newlist) != len(self._node_display_list):
            for item in newlist:
                if not item in self._node_display_list:
                    self.node_list.addItem(item)
        self._node_display_list = list(set(newlist + self._node_display_list))
