import os
import roslib
import rospy
roslib.load_manifest('rqt_console')

import qt_gui.qt_binding_helper  # @UnusedImport
from qt_gui.plugin import Plugin
from QtCore import QMutex, QTimer

from message_data_model import MessageDataModel
from message_proxy_model import MessageProxyModel
from main_window_widget import MainWindow
from custom_widgets import SetupDialog

class Console(Plugin):
    def __init__(self, context):
        super(Console, self).__init__(context)
        self.setObjectName('Console')

        self._datamodel = MessageDataModel()
        self._proxymodel = MessageProxyModel()
        self._proxymodel.setSourceModel(self._datamodel)

        self._mainwindow = MainWindow(self._proxymodel)
        context.add_widget(self._mainwindow)

        self._setupdialog = SetupDialog(context, self.message_callback)
        self._msgs = []
        
        self._mutex = QMutex()
        self._timer = QTimer()
        self._timer.timeout.connect(self.insert_messages)
        self._timer.start(100)

    def insert_messages(self):
        self._mutex.lock()
        msgs = self._msgs
        self._msgs=[]
        self._mutex.unlock()
        self._datamodel.insert_rows(msgs)
        self._mainwindow.update_status()

    def message_callback(self, msg):
        if not self._mainwindow.is_paused():
            self._mutex.lock()
            self._msgs.append(msg)     
            self._mutex.unlock()

    def shutdown_plugin(self):
        self._setupdialog.unsub_topic()
        self._setupdialog.close()
        self._timer.stop()

    def save_settings(self, plugin_settings, instance_settings):
        for index, member in enumerate(self._datamodel.message_members()):
            instance_settings.set_value(member,self._proxymodel.get_filter(index))

    def restore_settings(self, plugin_settings, instance_settings):
        for index, member in enumerate(self._datamodel.message_members()):
            text = instance_settings.value(member)
            if type(text) is type(None):
                text=''
            self._proxymodel.set_filter(index, text)

    def trigger_configuration(self):
        self._setupdialog.refresh_nodes()
        self._setupdialog.node_list.item(0).setSelected(True)
        self._setupdialog.node_changed(0)
        self._setupdialog.exec_()

