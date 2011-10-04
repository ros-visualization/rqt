from QtBindingHelper import import_from_qt
QObject = import_from_qt('QObject', 'QtCore')

class Plugin(QObject):

    def __init__(self, plugin_context):
        QObject.__init__(self)
        self.setObjectName('Plugin')

    def close_plugin(self):
        '''Close the plugin'''
        raise NotImplementedError('override method in subclass')

    def save_settings(self, global_settings, perspective_settings):
        '''Save intrinsic configuration to settings'''
        pass

    def restore_settings(self, global_settings, perspective_settings):
        '''Restore intrinsic configuration from settings'''
        pass
