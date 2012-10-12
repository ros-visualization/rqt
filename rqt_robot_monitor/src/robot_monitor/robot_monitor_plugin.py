import roslib;roslib.load_manifest('robot_monitor')
import rospy

from robot_monitor import RobotMonitor
from qt_gui.plugin import Plugin

class RobotMonitorPlugin(Plugin):
    def __init__(self, context):
        super(RobotMonitorPlugin, self).__init__(context)
        context.add_widget(RobotMonitor('/diagnostics_agg'))

        self.setObjectName('Robot Monitor')

