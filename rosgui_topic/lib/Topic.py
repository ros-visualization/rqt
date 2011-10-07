#!/usr/bin/env python
# -*- coding: utf-8 -*-
from rosgui.QtBindingHelper import import_from_qt
QObject, Qt = import_from_qt(['QObject', 'Qt'], 'QtCore')
QDockWidget = import_from_qt('QDockWidget', 'QtGui')

import roslib
roslib.load_manifest('rosgui_topic')

import TopicWidget
reload(TopicWidget) # force reload to update on changes during runtime

# main class inherits from the ui window class
class Topic(QObject):

    def __init__(self, parent, plugin_context):
        QObject.__init__(self, parent)
        self.setObjectName('Topic')

        self.widget = TopicWidget.TopicWidget(self, plugin_context)

        # add widget to the main window
        plugin_context.main_window().addDockWidget(Qt.RightDockWidgetArea, self.widget)


    def set_name(self, name):
        self.widget.setWindowTitle(name) 


    def close_plugin(self):
        QDockWidget.close(self.widget)
        self.widget.deleteLater()
