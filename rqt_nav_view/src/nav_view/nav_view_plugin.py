import roslib;roslib.load_manifest('nav_view')
import rospy

from nav_view import NavViewWidget
from qt_gui.plugin import Plugin

class NavViewPlugin(Plugin):
    def __init__(self, context):
        super(NavViewPlugin, self).__init__(context)
        context.add_widget(NavViewWidget())

        self.setObjectName('Naviation View')

