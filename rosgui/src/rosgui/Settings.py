import QtBindingHelper
from QtCore import QObject, Slot

class Settings(QObject):

    def __init__(self, settings_proxy, group):
        QObject.__init__(self)
        self.setObjectName('Settings')

        self.settings_proxy_ = settings_proxy
        self.group_ = group

    def get_settings(self, group):
        return Settings(self.settings_proxy_, self.group_ + '/' + group)

    def all_keys(self):
        return self.settings_proxy_.all_keys(self.group_)

#    def begin_read_array(self):

#    def begin_write_array(self):

    def child_groups(self):
        return self.settings_proxy_.child_groups(self.group_)

    def child_keys(self):
        return self.settings_proxy_.child_keys(self.group_)

    @Slot(str, result = bool)
    def contains(self, key):
        return self.settings_proxy_.contains(self.group_, key)

#    def end_array(self):

    @Slot(str)
    def remove(self, key):
        self.settings_proxy_.remove(self.group_, key)

#    def set_array_index(self, i):

    @Slot(str, 'QVariant')
    def set_value(self, key, value):
        self.settings_proxy_.set_value(self.group_, key, value)

    @Slot(str, 'QVariant', result = 'QVariant')
    def value(self, key, defaultValue=None):
        return self.settings_proxy_.value(self.group_, key, defaultValue)
