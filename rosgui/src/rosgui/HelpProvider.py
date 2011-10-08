import webbrowser
import QtBindingHelper
from QtCore import Slot, QObject

class HelpProvider(QObject):

    @Slot(object)
    def plugin_help_request(self, plugin_descriptor):
        plugin_attributes = plugin_descriptor.attributes()
        webbrowser.open('http://www.ros.org/wiki/fuerte/Planning/ROS%20GUI/plugins/' + plugin_attributes['library_name'])
