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

from python_qt_binding.QtCore import QMutex, QTimer

from qt_gui.plugin import Plugin

from .console_subscriber import ConsoleSubscriber
from .console_widget import ConsoleWidget
from .message_data_model import MessageDataModel
from .message_proxy_model import MessageProxyModel


class Console(Plugin):
    """
    rqt_console plugin's main class. Handles communication with ros_gui and contains
    callbacks to handle incoming message
    """
    def __init__(self, context):
        """
        :param context: plugin context hook to enable adding widgets as a ROS_GUI pane, ''PluginContext''
        """
        super(Console, self).__init__(context)
        self.setObjectName('Console')

        self._datamodel = MessageDataModel()
        self._proxymodel = MessageProxyModel()
        self._proxymodel.setSourceModel(self._datamodel)

        self._mainwindow = ConsoleWidget(self._proxymodel)
        if context.serial_number() > 1:
            self._mainwindow.setWindowTitle(self._mainwindow.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._mainwindow)

        self._consolesubscriber = ConsoleSubscriber(self.message_callback)

        # Timer and Mutex for flushing recieved messages to datamodel.
        # Required since QSortProxyModel can not handle a high insert rate
        self._mutex = QMutex()
        self._timer = QTimer()
        self._timer.timeout.connect(self.insert_messages)
        self._timer.start(100)

    def insert_messages(self):
        """
        Callback for flushing incoming Log messages from the queue to the datamodel
        """
        self._mutex.lock()
        msgs = self._datamodel._insert_message_queue
        self._datamodel._insert_message_queue = []
        self._mutex.unlock()
        self._datamodel.insert_rows(msgs)
        self._mainwindow.update_status()

    def message_callback(self, msg):
        """
        Callback for adding an incomming message to the queue
        """
        if not self._datamodel._paused:
            self._mutex.lock()
            self._datamodel._insert_message_queue.append(msg)
            self._mutex.unlock()

    def shutdown_plugin(self):
        self._consolesubscriber.unsubscribe_topic()
        self._timer.stop()
        self._mainwindow.cleanup_browsers_on_close()

    def save_settings(self, plugin_settings, instance_settings):
        self._mainwindow.save_settings(plugin_settings, instance_settings)

    def restore_settings(self, plugin_settings, instance_settings):
        self._mainwindow.restore_settings(plugin_settings, instance_settings)

    def trigger_configuration(self):
        self._consolesubscriber.set_message_limit(self._datamodel._message_limit)
        ok = self._consolesubscriber.show_dialog()
        if ok:
            self._datamodel._message_limit = self._consolesubscriber.get_message_limit()
