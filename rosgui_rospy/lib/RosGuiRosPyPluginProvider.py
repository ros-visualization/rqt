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
        super(RosGuiRosPyPluginProvider, self).__init__([ActualRosPluginProvider('rosgui', 'rosgui_rospy::Plugin')])
        self.setObjectName('RosGuiRosPyPluginProvider')

    def discover(self):
        # initialize ROS node
        rospy.init_node('rosgui_rospy_node', argv=None, anonymous=True, log_level=rospy.INFO, disable_rostime=False, disable_rosout=False, disable_signals=True)

        return CompositePluginProvider.discover(self)
