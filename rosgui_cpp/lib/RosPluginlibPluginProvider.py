from rosgui.PluginDescriptor import PluginDescriptor
from rosgui.PluginProvider import PluginProvider

import CppBindingHelper #@UnusedImport
from rosgui_cpp import rosgui_cpp

class RosPluginlibPluginProvider(PluginProvider):

    def __init__(self, plugin_provider):
        super(RosPluginlibPluginProvider, self).__init__()
        self.plugin_provider_ = plugin_provider

    def discover(self):
        discovered_plugins = self.__unfold(self.plugin_provider_.discover())
        plugin_descriptors = []
        for plugin in discovered_plugins.values():
            plugin_descriptor = PluginDescriptor(plugin['plugin_id'], plugin['attributes'])

            action_attributes = plugin['action']
            plugin_descriptor.set_action_attributes(action_attributes['label'], action_attributes.get('statustip', None), action_attributes.get('icon', None), action_attributes.get('icontype', None))

            groups = plugin['groups'] if 'groups' in plugin else {}
            for group in groups.values():
                plugin_descriptor.add_group_attributes(group['label'], group['statustip'], group['icon'], group['icontype'])

            plugin_descriptors.append(plugin_descriptor)
        return plugin_descriptors

    def load(self, plugin_id, plugin_context):
        cpp_plugin_context = None
        main_window = rosgui_cpp.MainWindowInterface(plugin_context.main_window())
        if plugin_context is not None:
            cpp_plugin_context = rosgui_cpp.PluginContext(main_window, plugin_context.serial_number())
            for key, value in plugin_context.attributes().items():
                cpp_plugin_context.set_attribute(key, value)
        bridge = rosgui_cpp.PluginBridge()
        loaded = bridge.load_plugin(self.plugin_provider_, plugin_id, cpp_plugin_context)
        if not loaded:
            return None
        main_window.set_plugin_instance(bridge)
        return bridge

    def unload(self, bridge):
        return bridge.unload_plugin()

    def __unfold(self, flat_dict):
        dictionary = {}
        for key, value in flat_dict.items():
            keys = str(key).split('.')
            current_level = dictionary
            for i in keys[:-1]:
                if i not in current_level:
                    current_level[i] = {}
                current_level = current_level[i]
            current_level[keys[-1]] = str(value) if value != '' else None
        return dictionary
