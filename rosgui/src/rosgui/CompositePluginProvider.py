import traceback

import QtBindingHelper #@UnusedImport
from QtCore import qCritical

from PluginProvider import PluginProvider

class CompositePluginProvider(PluginProvider):

    def __init__(self, plugin_providers=None):
        super(CompositePluginProvider, self).__init__()
        self.setObjectName('CompositePluginProvider')

        self._plugin_providers = plugin_providers or []
        self._discovered_plugins = {}
        self._running_plugins = {}

    def set_plugin_providers(self, plugin_providers):
        self._plugin_providers = plugin_providers

    def discover(self):
        # discover plugins from all providers
        discovered_plugins = []
        for plugin_provider in self._plugin_providers:
            try:
                plugin_descriptors = plugin_provider.discover()
            except Exception:
                qCritical('CompositePluginProvider.discover() could not discover plugins from provider "%s":\n%s' % (type(plugin_provider), traceback.format_exc()))
            else:
                self._discovered_plugins[plugin_provider] = plugin_descriptors
                discovered_plugins += plugin_descriptors
        return discovered_plugins

    def load(self, plugin_id, plugin_context):
        # dispatch load to appropriate provider
        for plugin_provider, plugin_descriptors in self._discovered_plugins.items():
            for plugin_descriptor in plugin_descriptors:
                if plugin_descriptor.plugin_id() == plugin_id:
                    instance = plugin_provider.load(plugin_id, plugin_context)
                    self._running_plugins[instance] = plugin_provider
                    return instance
        raise UserWarning('plugin_id "%s" not found' % plugin_id)

    def unload(self, plugin_instance):
        # dispatch unload to appropriate provider
        if plugin_instance in self._running_plugins:
            self._running_plugins[plugin_instance].unload(plugin_instance)
            self._running_plugins.pop(plugin_instance)
            return
        raise UserWarning('plugin_instance not found')
