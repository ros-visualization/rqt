import QtBindingHelper #@UnusedImport
from QtCore import QObject, Slot

class PluginContext(QObject):

    def __init__(self, parent=None):
        super(PluginContext, self).__init__(parent)
        self.setObjectName('PluginContext')

        self.main_window_ = None
        self.serial_number_ = 0
        self.dict_ = {}

    @Slot(result=object)
    def main_window(self):
        return self.main_window_

    def set_main_window(self, main_window):
        self.main_window_ = main_window

    @Slot(result=str)
    def serial_number(self):
        return self.serial_number_

    def set_serial_number(self, serial_number):
        self.serial_number_ = serial_number

    def attributes(self):
        return self.dict_

    def attribute(self, key):
        if self.dict_.has_key(key):
            return self.dict_[key]
        return None

    def set_attribute(self, key, value):
        self.dict_[key] = value
