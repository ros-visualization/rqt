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
import rospy
import rospkg
import rosmsg
import roslib

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtGui import QAction, QIcon, QMenu, QMessageBox, QTreeView, QWidget

from .messages_tree_model import MessagesTreeModel
from .messages_tree_view import MessagesTreeView

from rqt_console.text_browse_dialog import TextBrowseDialog


class MessagesWidget(QWidget):
    def __init__(self, mode=rosmsg.MODE_MSG):
        super(MessagesWidget, self).__init__()
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'messages.ui')
        loadUi(ui_file, self, {'MessagesTreeView': MessagesTreeView})
        self.setObjectName('MessagesUi')
        self._mode = mode

        self.add_button.setIcon(QIcon.fromTheme('list-add'))
        self.add_button.clicked.connect(self._add_message)
        self._refresh_packages()
        self._refresh_messages(self.package_combo.itemText(0))
        self.package_combo.currentIndexChanged[str].connect(self._refresh_messages)
        self.messages_tree.mousePressEvent = self._handle_mouse_press

        self._browsers = []

    def _refresh_packages(self):
        rospack = rospkg.RosPack()
        packages = sorted([pkg_tuple[0] for pkg_tuple in rosmsg.iterate_packages(rospack, self._mode)])
        self._package_list = packages
        self.package_combo.clear()
        self.package_combo.addItems(self._package_list)
        self.package_combo.setCurrentIndex(0)

    def _refresh_messages(self, package=None):
        if package is None or len(package) == 0:
            return
        self._messages = []
        if self._mode == rosmsg.MODE_MSG:
            message_list = rosmsg.list_msgs(package)
        elif self._mode == rosmsg.MODE_SRV:
            message_list = rosmsg.list_srvs(package)
        for message in message_list:
            if self._mode == rosmsg.MODE_MSG:
                message_class = roslib.message.get_message_class(message)
            elif self._mode == rosmsg.MODE_SRV:
                message_class = roslib.message.get_service_class(message)
            if message_class is not None:
                self._messages.append(message)

        self._messages = [ x.split('/')[1] for x in self._messages]

        self.message_combo.clear()
        self.message_combo.addItems(self._messages)

    def _add_message(self):
        if self.message_combo.count() == 0:
            return
        message = self.package_combo.currentText() + '/' + self.message_combo.currentText()
        if self._mode == rosmsg.MODE_MSG:
            message_class = roslib.message.get_message_class(message)()
            self.messages_tree.model().add_message(message_class, self.tr('Message Root'), message, message)
        elif self._mode == rosmsg.MODE_SRV:
            message_class = roslib.message.get_service_class(message)()
            self.messages_tree.model().add_message(message_class._request_class, self.tr('Service Request'), message, message)
            self.messages_tree.model().add_message(message_class._response_class, self.tr('Service Response'), message, message)
        self.messages_tree._recursive_set_editable(self.messages_tree.model().invisibleRootItem(), False)

    def _handle_mouse_press(self, event, old_pressEvent=QTreeView.mousePressEvent):
        if event.buttons() & Qt.RightButton and event.modifiers() == Qt.NoModifier:
            self._rightclick_menu(event)
            event.accept()
        return old_pressEvent(self.messages_tree, event)

    def _rightclick_menu(self, event):
        menu = QMenu()
        text_action = QAction(self.tr('View Text'), menu)
        menu.addAction(text_action)
        raw_action = QAction(self.tr('View Raw'), menu)
        menu.addAction(raw_action)

        action = menu.exec_(event.globalPos())
        
        if action == raw_action or action == text_action:
            selected = self.messages_tree.selectedIndexes()
            selected_type = selected[1].data()
            
            if selected_type[-2:] == '[]':
                selected_type = selected_type[:-2]
            browsetext = None
            try:
                if self._mode == rosmsg.MODE_MSG:
                    browsetext = rosmsg.get_msg_text(selected_type, action == raw_action)
                elif self._mode == rosmsg.MODE_SRV:
                    browsetext = rosmsg.get_srv_text(selected_type, action == raw_action)
                else:
                    raise
            except rosmsg.ROSMsgException, e:
                QMessageBox.warning(self, self.tr('Warning'), self.tr('The selected item component does not have text to view.'))
            if browsetext is not None:
                self._browsers.append(TextBrowseDialog(browsetext))
                self._browsers[-1].show()
        else:
            return

    def cleanup_browsers_on_close(self):
        for browser in self._browsers:
            browser.close()

