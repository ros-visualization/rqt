from QtBindingHelper import import_from_qt
QObject = import_from_qt('QObject', 'QtCore')

class PluginProvider(QObject):

    def __init__(self):
        QObject.__init__(self)
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
