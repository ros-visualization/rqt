import QtBindingHelper #@UnusedImport
from QtCore import QObject, Slot

class PluginContext(QObject):

    def __init__(self, parent=None):
        super(PluginContext, self).__init__(parent)
        self.setObjectName('PluginContext')

        self._main_window = None
        self._serial_number = 0
        self._dict = {}

    @Slot(result=object)
    def main_window(self):
        return self._main_window

    def set_main_window(self, main_window):
        self._main_window = main_window

    @Slot(result=int)
    def serial_number(self):
        return self._serial_number

    def set_serial_number(self, serial_number):
        self._serial_number = serial_number

    def attributes(self):
        return self._dict

    def attribute(self, key):
        return self._dict.get(key, None)

    def set_attribute(self, key, value):
        self._dict[key] = value
