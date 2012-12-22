#!/usr/bin/env python
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
#  * Neither the name of the Willow Garage nor the names of its
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
import rospy
import rospkg
import os

from python_qt_binding import loadUi
from python_qt_binding.QtGui import QIcon, QTreeWidgetItem, QWidget
from python_qt_binding.QtCore import Qt, QTimer, QObject

from diagnostic_msgs.msg import DiagnosticArray

import threading
import cStringIO
import copy


class TreeItem(QObject):
    ##\param status DiagnosticsStatus : Diagnostic data of item
    ##\param tree_node wxTreeItemId : Tree ID of item in display
    ##\param stamp ros::Time : Stamp when message was received
    def __init__(self, status, tree_node, stamp=None):
        super(TreeItem, self).__init__()
        self.status = status
        self.mark = False
        self.stale = False
        self.tree_node = tree_node
        self.stamp = stamp


class RuntimeMonitorWidget(QWidget):
    def __init__(self, topic="/diagnostics"):
        super(RuntimeMonitorWidget, self).__init__()
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_runtime_monitor'), 'resource', 'runtime_monitor_widget.ui')
        loadUi(ui_file, self)
        self.setObjectName('RuntimeMonitorWidget')

        self._mutex = threading.Lock()

        self._error_icon = QIcon.fromTheme('dialog-error')
        self._warning_icon = QIcon.fromTheme('dialog-warning')
        self._ok_icon = QIcon.fromTheme('dialog-information')

        self._stale_node = QTreeWidgetItem(self.tree_widget.invisibleRootItem(), ['Stale (0)'])
        self._stale_node.setIcon(0, self._error_icon)
        self.tree_widget.addTopLevelItem(self._stale_node)

        self._error_node = QTreeWidgetItem(self.tree_widget.invisibleRootItem(), ['Errors (0)'])
        self._error_node.setIcon(0, self._error_icon)
        self.tree_widget.addTopLevelItem(self._error_node)

        self._warning_node = QTreeWidgetItem(self.tree_widget.invisibleRootItem(), ['Warnings (0)'])
        self._warning_node.setIcon(0, self._warning_icon)
        self.tree_widget.addTopLevelItem(self._warning_node)

        self._ok_node = QTreeWidgetItem(self.tree_widget.invisibleRootItem(), ['Ok (0)'])
        self._ok_node.setIcon(0, self._ok_icon)
        self.tree_widget.addTopLevelItem(self._ok_node)
        self.tree_widget.itemSelectionChanged.connect(self._on_item_selected)
        self.keyPressEvent = self._on_key_press

        self._name_to_item = {}
        self._new_errors_callback = None

        self._subscriber = rospy.Subscriber(topic, DiagnosticArray, self._diagnostics_callback)
        self._timer = QTimer()
        self._timer.timeout.connect(self._on_timer)
        self._timer.start(5000)

        self._messages = []
        self._used_items = 0

    def __del__(self):
        if self._subscriber is not None:
            self._subscriber.unregister()
            self._subscriber = None

    def shutdown(self):
        """
        Unregisters diagnostics subscriber for clean shutdown
        """
        if rospy.is_shutdown():
            return

        if self._subscriber is not None:
            self._subscriber.unregister()
            self._subscriber = None

    def change_diagnostic_topic(self, topic):
        """
        Changes diagnostics topic name. Must be of type diagnostic_msgs/DiagnosticArray
        """
        if len(topic) == 0:
            self.reset_monitor()
            return

        if self._subscriber is not None:
            self._subscriber.unregister()
            self._subscriber = rospy.Subscriber(str(topic), DiagnosticArray, self._diagnostics_callback)
        self.reset_monitor()

    def reset_monitor(self):
        """
        Removes all values from monitor display, resets buffers
        """
        self._name_to_item = {}  # Reset all stale topics
        self._messages = []
        self._clear_tree()

    def _clear_tree(self):
        for index in range(self._stale_node.childCount()):
            self._stale_node.removeChild(self._stale_node.child(index))
        for index in range(self._error_node.childCount()):
            self._error_node.removeChild(self._error_node.child(index))
        for index in range(self._warning_node.childCount()):
            self._warning_node.removeChild(self._warning_node.child(index))
        for index in range(self._ok_node.childCount()):
            self._ok_node.removeChild(self._ok_node.child(index))
        self._update_root_labels()

    def _diagnostics_callback(self, message):
        with self._mutex:
            self._messages.append(message)

        self.new_message(rospy.get_rostime())

    def new_message(self, stamp=None):
        with self._mutex:
            had_errors = False

            for message in self._messages:
                for status in message.status:
                    was_selected = False
                    if (self._name_to_item.has_key(status.name)):
                        item = self._name_to_item[status.name]
                        if self.tree_widget.selectedItems() != [] and self.tree_widget.selectedItems()[0] == item:
                            was_selected = True
                        if (item.status.level == 2 and status.level != 2):
                            had_errors = True
                        self._update_item(item, status, was_selected, stamp)
                    else:
                        self._create_item(status, was_selected, True, stamp)
                        if (status.level == 2):
                            had_errors = True
            self._messages = []

        if (had_errors and self._new_errors_callback != None):
            self._new_errors_callback()

        self._update_root_labels()
        self.update()

    def _update_item(self, item, status, was_selected, stamp):
        change_parent = False
        if (item.status.level != status.level):
            change_parent = True
        if (change_parent):
            if (item.status.level == 0):
                self._ok_node.removeChild(item.tree_node)
            elif (item.status.level == 1):
                self._warning_node.removeChild(item.tree_node)
            elif (item.status.level == -1):
                self._stale_node.removeChild(item.tree_node)
            else:
                self._error_node.removeChild(item.tree_node)

            if (status.level == 0):
                parent_node = self._ok_node
            elif (status.level == 1):
                parent_node = self._warning_node
            elif (status.level == -1):
                parent_node = self._stale_node
            else:
                parent_node = self._error_node

            item.tree_node.setText(0, status.name + ": " + status.message)
            item.stamp = stamp
            item.tree_node.setData(0, Qt.UserRole, item)
            parent_node.addChild(item.tree_node)

            if (status.level > 1 or status.level == -1):
                parent_node.setExpanded(True)

            parent_node.sortChildren(0, Qt.AscendingOrder)

            if (was_selected):
                item.tree_node.setSelected(True)

        else:
            item.tree_node.setText(0, status.name + ": " + status.message)

        item.status = status

        if (was_selected):
            self._fillout_info(item.tree_node)

        item.mark = True

    def _create_item(self, status, select, expand_if_error, stamp):
        if (status.level == 0):
            parent_node = self._ok_node
        elif (status.level == 1):
            parent_node = self._warning_node
        elif (status.level == -1):
            parent_node = self._stale_node
        else:
            parent_node = self._error_node

        item = TreeItem(status, QTreeWidgetItem(parent_node, [status.name + ": " + status.message]), stamp)
        item.tree_node.setData(0, Qt.UserRole, item)
        parent_node.addChild(item.tree_node)

        self._name_to_item[status.name] = item

        parent_node.sortChildren(0, Qt.AscendingOrder)

        if (select):
            item.tree_node.setSelected(True)

        if (expand_if_error and (status.level > 1 or status.level == -1)):
            parent_node.setExpanded(True)

        item.mark = True

        return item

    def _fillout_info(self, node):
        item = node.data(0, Qt.UserRole)
        if (item == None):
            return

        status = item.status

        s = cStringIO.StringIO()

        s.write("<html><body>")
        s.write("<b>Component</b>: %s<br>\n" % (status.name))
        s.write("<b>Message</b>: %s<br>\n" % (status.message))
        s.write("<b>Hardware ID</b>: %s<br><br>\n\n" % (status.hardware_id))

        s.write('<table border="1" cellpadding="2" cellspacing="0">')
        for value in status.values:
            value.value = value.value.replace("\n", "<br>")
            s.write("<tr><td><b>%s</b></td> <td>%s</td></tr>\n" % (value.key, value.value))

        s.write("</table></body></html>")

        self.html_browser.setHtml(s.getvalue())

    def _on_item_selected(self):
        current_item = self.tree_widget.selectedItems()
        if current_item is not None:
            self._fillout_info(current_item[0])

    def _on_key_press(self, event):
        key = event.key()
        if key == Qt.Key_Delete:
            nodes = self.tree_widget.selectedItems()
            if (nodes != [] and nodes[0] not in (self._ok_node, self._warning_node, self._stale_node, self._error_node)):
                item = nodes[0].data(0, Qt.UserRole)
                if (item.status.level == 0):
                    self._ok_node.removeChild(item.tree_node)
                elif (item.status.level == 1):
                    self._warning_node.removeChild(item.tree_node)
                elif (item.status.level == -1):
                    self._stale_node.removeChild(item.tree_node)
                else:
                    self._error_node.removeChild(item.tree_node)
                del self._name_to_item[item.status.name]
        else:
            event.Skip()
        self._update_root_labels()
        self.update()

    def _on_timer(self):
        for name, item in self._name_to_item.iteritems():
            node = item.tree_node
            if (item != None):
                if (not item.mark):
                    was_selected = False
                    selected = self.tree_widget.selectedItems()
                    if selected != [] and selected[0] == node:
                        was_selected = True

                    new_status = copy.deepcopy(item.status)
                    new_status.level = -1
                    self._update_item(item, new_status, was_selected, item.stamp)
                item.mark = False
        self._update_root_labels()
        self.update()

    def set_new_errors_callback(self, callback):
        self._new_errors_callback = callback

    def get_num_errors(self):
        return self._error_node.childCount() + self._stale_node.childCount()

    def get_num_warnings(self):
        return self._warning_node.childCount()

    def get_num_ok(self):
        return self._ok_node.childCount()

    def _update_root_labels(self):
        self._stale_node.setText(0, "Stale (%s)" % (self._stale_node.childCount()))
        self._error_node.setText(0, "Errors (%s)" % (self._error_node.childCount()))
        self._warning_node.setText(0, "Warnings (%s)" % (self._warning_node.childCount()))
        self._ok_node.setText(0, "Ok (%s)" % (self._ok_node.childCount()))
