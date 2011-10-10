import webbrowser

import QtBindingHelper #@UnusedImport
from QtCore import QObject, Slot

class HelpProvider(QObject):

    def __init__(self):
        super(HelpProvider, self).__init__()

    @Slot(object)
    def plugin_help_request(self, plugin_descriptor):
        plugin_attributes = plugin_descriptor.attributes()
        webbrowser.open('http://www.ros.org/wiki/fuerte/Planning/ROS%20GUI/plugins/' + plugin_attributes['library_name'])
