import QtBindingHelper #@UnusedImport
from QtCore import QObject

class PluginProvider(QObject):

    def __init__(self):
        super(PluginProvider, self).__init__()
        self.setObjectName('PluginProvider')

    def discover(self):
        '''Discover the plugins; returns a dictionary'''
        raise NotImplementedError('override method in subclass')

    def load(self, plugin_id, plugin_context):
        '''Load a plugin; returns an identifier of the instance'''
        raise NotImplementedError('override method in subclass')

    def unload(self, plugin_instance):
        '''Unload a plugin'''
        raise NotImplementedError('override method in subclass')
