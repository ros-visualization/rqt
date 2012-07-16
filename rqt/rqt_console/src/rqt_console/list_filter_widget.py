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


class ListFilterWidget(QWidget):
    def __init__(self, parentfilter, display_list_args):
        super(ListFilterWidget, self).__init__()
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'list_filter_widget.ui')
        loadUi(ui_file, self)
        self.setObjectName('ListFilterWidget')
        self._parentfilter = parentfilter  # When data is changed we need to store it in the parent filter
        
        self._list_populate_function = display_list_args[0]
        self._function_argument = False
        if len(display_list_args) > 1:
            self._function_argument = display_list_args[1]
        self.list_widget.itemSelectionChanged.connect(self.handle_item_changed)
        self._display_list = [] 
        
        self.repopulate()
    
    def select_item(self, item):
        items = self.list_widget.findItems(item, Qt.MatchExactly)
        for item in items:
            item.setSelected(True)
        self.handle_item_changed()

    def handle_item_changed(self):
        self._parentfilter.set_list(self.list_widget.selectedItems())

    def repopulate(self):
        if not self._function_argument is False:
            newlist =  self._list_populate_function(self._function_argument)
        else:
            newlist =  self._list_populate_function()

        if len(newlist) != len(self._display_list):
            for item in newlist:
                if not item in self._display_list:
                    self.list_widget.addItem(item)
        self._display_list = list(set(newlist + self._display_list))
