import os, sys

from rosgui.CompositePluginProvider import CompositePluginProvider

# append folder of this file to module search path
sys.path.append(os.path.realpath(os.path.dirname(__file__)))
from CppBindingHelper import ROSGUI_CPP_BINDING
from RosPluginlibPluginProvider import RosPluginlibPluginProvider

class CppPluginProvider(CompositePluginProvider):

    def __init__(self):
        plugin_providers = [
            RosPluginlibPluginProvider(ROSGUI_CPP_BINDING.RosPluginlibPluginProvider_ForPlugins('rosgui', 'rosgui_cpp::Plugin')),
            RosPluginlibPluginProvider(ROSGUI_CPP_BINDING.RecursivePluginProvider(ROSGUI_CPP_BINDING.RosPluginlibPluginProvider_ForPluginProviders.create_instance('rosgui', 'rosgui_cpp::PluginProvider'))),
        ]
        CompositePluginProvider.__init__(self, plugin_providers)
