import os
import roslib
roslib.load_manifest('rqt_logger_level')
import rospy

from qt_gui.plugin import Plugin
from qt_gui.qt_binding_helper import loadUi
from QtGui import QWidget

from logger_level_widget import LoggerLevelWidget
from logger_level_service_caller import LoggerLevelServiceCaller

class LoggerLevel(Plugin):

    def __init__(self, context):
        super(LoggerLevel, self).__init__(context)
        self.setObjectName('LoggerLevel')
        self._service_caller = LoggerLevelServiceCaller()
        self._widget = LoggerLevelWidget(self._service_caller)

        context.add_widget(self._widget)

    def shutdown_plugin(self):
        pass

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure it
        # Usually used to open a dialog to offer the user a set of configuration
