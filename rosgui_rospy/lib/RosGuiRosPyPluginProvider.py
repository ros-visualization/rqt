import roslib
roslib.load_manifest('rosgui_rospy')
import rospy

from rosgui.CompositePluginProvider import CompositePluginProvider

try:
    from rosgui.RospkgPluginProvider import RospkgPluginProvider
    ActualRosPluginProvider = RospkgPluginProvider
except ImportError:
    from rosgui.RoslibPluginProvider import RoslibPluginProvider
    ActualRosPluginProvider = RoslibPluginProvider

class RosGuiRosPyPluginProvider(CompositePluginProvider):

    def __init__(self):
        CompositePluginProvider.__init__(self, [ActualRosPluginProvider('rosgui', 'rosgui_rospy::Plugin')])
        self.setObjectName('RosGuiRosPyPluginProvider')

    def discover(self):
        # initialize ROS node
        rospy.init_node('rosgui_rospy_node', None, True, rospy.INFO, False, False, True)

        return CompositePluginProvider.discover(self)
