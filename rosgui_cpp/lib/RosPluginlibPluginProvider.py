from rosgui.PluginDescriptor import PluginDescriptor
from rosgui.PluginProvider import PluginProvider
from rosgui.QtBindingHelper import import_from_qt
QObject = import_from_qt('QObject', 'QtCore')

from CppBindingHelper import ROSGUI_CPP_BINDING


class RosPluginlibPluginProvider(PluginProvider):

    def __init__(self, plugin_provider):
        PluginProvider.__init__(self)
        self.plugin_provider_ = plugin_provider
        self.instances_ = {}

    def discover(self):
        discovered_plugins = self.__unfold(self.plugin_provider_.discover())
        plugin_descriptors = []
        for plugin in discovered_plugins.values():
            plugin_descriptor = PluginDescriptor(plugin['plugin_id'], plugin['attributes'])

            action_attributes = plugin['action']
            plugin_descriptor.set_action_attributes(action_attributes['label'], action_attributes.get('statustip', None), action_attributes.get('icon', None), action_attributes.get('icontype', None))

            groups = plugin['groups'] if plugin.has_key('groups') else {}
            for group in groups.values():
                plugin_descriptor.add_group_attributes(group['label'], group['statustip'], group['icon'], group['icontype'])

            plugin_descriptors.append(plugin_descriptor)
        return plugin_descriptors

    def load(self, plugin_id, plugin_context):
        cpp_plugin_context = None
        main_window = ROSGUI_CPP_BINDING.MainWindowInterface.create_instance(plugin_context.main_window())
        if plugin_context is not None:
            cpp_plugin_context = ROSGUI_CPP_BINDING.PluginContext(main_window, str(plugin_context.serial_number()))
            for key, value in plugin_context.attributes().items():
                cpp_plugin_context.set_attribute(key, value)
        instance = self.plugin_provider_.load_plugin(plugin_id, cpp_plugin_context)
        if instance is None:
            return None
        bridge = ROSGUI_CPP_BINDING.PluginBridge(instance)
        main_window.set_plugin_instance(bridge)
        self.instances_[bridge] = instance
        return bridge

    def unload(self, plugin_instance):
        instance = self.instances_.pop(plugin_instance)
        return self.plugin_provider_.unload_plugin(instance)

    def __unfold(self, flat_dict):
        dictionary = {}
        for key, value in flat_dict.items():
            keys = str(key).split('.')
            current_level = dictionary
            for i in keys[:-1]:
                if not current_level.has_key(i):
                    current_level[i] = {}
                current_level = current_level[i]
            current_level[keys[-1]] = str(value) if value != '' else None
        return dictionary
