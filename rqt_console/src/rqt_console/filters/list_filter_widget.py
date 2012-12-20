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
import rospkg

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtGui import QWidget

from rqt_py_common.ini_helper import pack, unpack


class ListFilterWidget(QWidget):
    """
    Generic List widget to be used when implementing filters that require
    limited dynamic selections
    """
    def __init__(self, parentfilter, display_list_args):
        """
        :param parentfilter: The filter object, must implement set_list and
        contain _list ''QObject''
        :param display_list_args: list of arguments which must contain a
        function designed to populate a list 'display_list_args[0]' and may
        contain an optional variable to pass into that function 'display_list_args[1]'
        """
        super(ListFilterWidget, self).__init__()
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_console'), 'resource/filters', 'list_filter_widget.ui')
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

    def select_item(self, text):
        """
        All items matching text will be selected in the list_widget
        :param item: a string to be matched against the list ''str''
        """
        items = self.list_widget.findItems(text, Qt.MatchExactly)
        for item in items:
            item.setSelected(True)
        self.handle_item_changed()

    def handle_item_changed(self):
        self._parentfilter.set_list(self.list_widget.selectedItems())

    def repopulate(self):
        """
        Repopulates the display widgets based on the function arguments passed
        in during initialization
        """
        if not self._function_argument is False:
            newlist = self._list_populate_function(self._function_argument)
        else:
            newlist = self._list_populate_function()

        if len(newlist) != len(self._display_list):
            for item in newlist:
                if item not in self._display_list:
                    self.list_widget.addItem(item)
        self._display_list = list(set(newlist + self._display_list))

    def save_settings(self, settings):
        """
        Saves the settings for this filter.
        :param settings: used to write the settings to an ini file ''qt_gui.settings.Settings''
        """
        settings.set_value('displist', pack(self._display_list))
        settings.set_value('itemlist', pack(self._parentfilter._list))

    def restore_settings(self, settings):
        """
        Restores the settings for this filter from an ini file.
        :param settings: used to extract the settings from an ini file ''qt_gui.settings.Settings''
        """
        self._display_list = unpack(settings.value('displist'))
        for item in self._display_list:
            if len(self.list_widget.findItems(item, Qt.MatchExactly)) == 0:
                self.list_widget.addItem(item)

        for index in range(self.list_widget.count()):
            self.list_widget.item(index).setSelected(False)
        item_list = unpack(settings.value('itemlist'))
        for item in item_list:
            self.select_item(item)
