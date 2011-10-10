import roslib

from RosPluginProvider import RosPluginProvider

class RoslibPluginProvider(RosPluginProvider):

    def __init__(self, export_tag, base_class_type):
        super(RoslibPluginProvider, self).__init__(export_tag, base_class_type)
        self.setObjectName('RoslibPluginProvider')

    def _find_rosgui_plugins(self):
        return roslib.rospack.rospack_plugins(self.export_tag_)
