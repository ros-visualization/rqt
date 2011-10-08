import traceback

import QtBindingHelper
from QtCore import qCritical

from CompositePluginProvider import CompositePluginProvider

class RecursivePluginProvider(CompositePluginProvider):

    def __init__(self, plugin_provider):
        CompositePluginProvider.__init__(self, [])
        self.setObjectName('RecursivePluginProvider')

        self.plugin_provider_ = plugin_provider

    def discover(self):
        # discover plugins, which are providers themselves
        plugin_descriptors = self.plugin_provider_.discover()

        # instantiate plugins
        plugin_providers = []
        for plugin_descriptor in plugin_descriptors:
            try:
                # pass None as PluginContext for PluginProviders
                instance = self.plugin_provider_.load(plugin_descriptor.plugin_id(), None)
            except Exception:
                qCritical('RecursivePluginProvider.discover() loading plugin "%s" failed:\n%s' % (str(plugin_descriptor.plugin_id()), traceback.format_exc()))
            else:
                if instance is not None:
                    plugin_providers.append(instance)

        # delegate discovery through instantiated plugin providers to base class
        self.set_plugin_providers(plugin_providers)
        return CompositePluginProvider.discover(self)
