import QtBindingHelper #@UnusedImport
from QtCore import QObject, Slot

class Settings(QObject):

    def __init__(self, settings_proxy, group):
        super(Settings, self).__init__()
        self.setObjectName('Settings')

        self._settings_proxy = settings_proxy
        self._group = group

    def get_settings(self, group):
        return Settings(self._settings_proxy, self._group + '/' + group)

    def all_keys(self):
        return self._settings_proxy.all_keys(self._group)

#    def begin_read_array(self):

#    def begin_write_array(self):

    def child_groups(self):
        return self._settings_proxy.child_groups(self._group)

    def child_keys(self):
        return self._settings_proxy.child_keys(self._group)

    @Slot(str, result=bool)
    def contains(self, key):
        return self._settings_proxy.contains(self._group, key)

#    def end_array(self):

    @Slot(str)
    def remove(self, key):
        self._settings_proxy.remove(self._group, key)

#    def set_array_index(self, i):

    @Slot(str, 'QVariant')
    def set_value(self, key, value):
        self._settings_proxy.set_value(self._group, key, value)

    @Slot(str, 'QVariant', result='QVariant')
    def value(self, key, defaultValue=None):
        return self._settings_proxy.value(self._group, key, defaultValue)

    def to_dict(self):
        keys = {}
        for key in self.child_keys():
            keys[str(key)] = self.value(key)
        groups = {}
        for group in self.child_groups():
            settings = self.get_settings(group)
            groups[str(group)] = settings.to_dict()
        return {'keys': keys, 'groups': groups}

    def from_dict(self, data):
        keys = data.get('keys', {})
        for key in keys:
            self.set_value(key, keys[key])
        groups = data.get('groups', {})
        for group in groups:
            settings = self.get_settings(group)
            settings.from_dict(groups[group])
