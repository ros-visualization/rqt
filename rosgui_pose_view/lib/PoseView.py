import rosgui.QtBindingHelper #@UnusedImport
from QtCore import QObject, Qt
from QtGui import QDockWidget

import roslib
roslib.load_manifest('rosgui_pose_view')

import PoseViewWidget
reload(PoseViewWidget) # force reload to update on changes during runtime

class PoseView(QObject):

    def __init__(self, parent, plugin_context):
        super(PoseView, self).__init__(parent)
        self.setObjectName('PoseView')

        self._widget = PoseViewWidget.PoseViewWidget(self, plugin_context)
        self.set_name('Pose View')

        if plugin_context.serial_number() != 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % plugin_context.serial_number()))

        # add _widget to the main window
        plugin_context.main_window().addDockWidget(Qt.RightDockWidgetArea, self._widget)


    def set_name(self, name):
        self._widget.setWindowTitle(name)


    def save_settings(self, global_settings, perspective_settings):
        self._widget.save_settings(global_settings, perspective_settings)


    def restore_settings(self, global_settings, perspective_settings):
        self._widget.restore_settings(global_settings, perspective_settings)


    def close_plugin(self):
        QDockWidget.close(self._widget)
        self._widget.deleteLater()
