import os
import roslib
import rospy
roslib.load_manifest('rqt_console')

import qt_gui.qt_binding_helper  # @UnusedImport
from qt_gui.plugin import Plugin
from QtCore import QMutex, QTimer
from QtGui import QWidget

from message_data_model import MessageDataModel
from message_proxy_model import MessageProxyModel
from console_widget import ConsoleWidget
from console_subscriber import ConsoleSubscriber

class Console(Plugin):
    def __init__(self, context):
        super(Console, self).__init__(context)
        self.setObjectName('Console')

        self._datamodel = MessageDataModel()
        self._proxymodel = MessageProxyModel()
        self._proxymodel.setSourceModel(self._datamodel)

        self._mainwindow = ConsoleWidget(self._proxymodel)
        context.add_widget(self._mainwindow)

        self._consolesubscriber = ConsoleSubscriber(self.message_callback)

        # Timer and Mutex for flushing recieved messages to datamodel.
        # Required since QSortProxyModel can not handle a high insert rate
        self._mutex = QMutex()
        self._timer = QTimer()
        self._timer.timeout.connect(self.insert_messages)
        self._timer.start(100)

    def insert_messages(self):
        self._mutex.lock()
        msgs = self._datamodel._insert_message_queue
        self._datamodel._insert_message_queue=[]
        self._mutex.unlock()
        self._datamodel.insert_rows(msgs)
        self._mainwindow.update_status()
    
    def message_callback(self, msg):
        if not self._datamodel._paused:
            self._mutex.lock()
            self._datamodel._insert_message_queue.append(msg)     
            self._mutex.unlock()

    def shutdown_plugin(self):
        self._consolesubscriber.unsubscribe_topic()
        self._timer.stop()

    def save_settings(self, plugin_settings, instance_settings):
        for index, member in enumerate(self._datamodel.message_members()):
            instance_settings.set_value(member,self._proxymodel.get_filter(index))

    def restore_settings(self, plugin_settings, instance_settings):
        for index, member in enumerate(self._datamodel.message_members()):
            text = instance_settings.value(member)
            if text is None:
                text=''
            self._proxymodel.set_filter(index, text)

    def trigger_configuration(self):
        self._consolesubscriber.show_dialog()

