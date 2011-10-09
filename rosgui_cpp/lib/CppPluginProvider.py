import os, sys

from rosgui.CompositePluginProvider import CompositePluginProvider

# append folder of this file to module search path
sys.path.append(os.path.realpath(os.path.dirname(__file__)))
import CppBindingHelper
from rosgui_cpp import rosgui_cpp
from RosPluginlibPluginProvider import RosPluginlibPluginProvider

class CppPluginProvider(CompositePluginProvider):

    def __init__(self):
        plugin_providers = [
            RosPluginlibPluginProvider(rosgui_cpp.RosPluginlibPluginProvider_ForPlugins('rosgui', 'rosgui_cpp::Plugin')),
            RosPluginlibPluginProvider(rosgui_cpp.RecursivePluginProvider(rosgui_cpp.RosPluginlibPluginProvider_ForPluginProviders.create_instance('rosgui', 'rosgui_cpp::PluginProvider'))),
        ]
        CompositePluginProvider.__init__(self, plugin_providers)
