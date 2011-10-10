#!/usr/bin/env python

import rosgui.QtBindingHelper #@UnusedImport
from QtCore import QObject, Qt
from QtGui import QDockWidget

import roslib
roslib.load_manifest('rosgui_topic')

import TopicWidget
reload(TopicWidget) # force reload to update on changes during runtime

# main class inherits from the ui window class
class Topic(QObject):

    def __init__(self, parent, plugin_context):
        super(Topic, self).__init__(parent)
        self.setObjectName('Topic')

        self.widget = TopicWidget.TopicWidget(self, plugin_context)

        if plugin_context.serial_number() != 1:
            self.widget.setWindowTitle(self.widget.windowTitle() + (' (%d)' % plugin_context.serial_number()))

        # add widget to the main window
        plugin_context.main_window().addDockWidget(Qt.RightDockWidgetArea, self.widget)


    def set_name(self, name):
        self.widget.setWindowTitle(name)


    def close_plugin(self):
        QDockWidget.close(self.widget)
        self.widget.deleteLater()
