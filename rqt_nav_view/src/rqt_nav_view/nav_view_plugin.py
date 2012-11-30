import roslib;roslib.load_manifest('rqt_nav_view')
import rospy

from .nav_view import NavViewWidget
from qt_gui.plugin import Plugin

class NavViewPlugin(Plugin):
    def __init__(self, context):
        super(NavViewPlugin, self).__init__(context)
        if context.serial_number() > 1:
            raise RuntimeError("Due to a limitation of tf_frames you may not run more than one instance of rqt_nav_view.")
        self._widget = NavViewWidget()
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

        self.setObjectName('Naviation View')

