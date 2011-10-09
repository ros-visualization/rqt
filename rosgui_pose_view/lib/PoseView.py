import rosgui.QtBindingHelper
from QtCore import QObject, Qt
from QtGui import QDockWidget

import roslib
roslib.load_manifest('rosgui_pose_view')

import PoseViewWidget
reload(PoseViewWidget) # force reload to update on changes during runtime

class PoseView(QObject):

    def __init__(self, parent, plugin_context):
        QObject.__init__(self, parent)
        self.setObjectName('PoseView')

        self.widget_ = PoseViewWidget.PoseViewWidget(self, plugin_context)
        self.set_name('Pose View')

        if plugin_context.serial_number() != 1:
            self.widget_.setWindowTitle(self.widget_.windowTitle() + (' (%d)' % plugin_context.serial_number()))

        # add widget to the main window
        plugin_context.main_window().addDockWidget(Qt.RightDockWidgetArea, self.widget_)


    def set_name(self, name):
        self.name_ = name
        self.widget_.setWindowTitle(name)


    def save_settings(self, global_settings, perspective_settings):
        self.widget_.save_settings(global_settings, perspective_settings)


    def restore_settings(self, global_settings, perspective_settings):
        self.widget_.restore_settings(global_settings, perspective_settings)


    def close_plugin(self):
        QDockWidget.close(self.widget_)
        self.widget_.deleteLater()
