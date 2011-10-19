import QtBindingHelper #@UnusedImport
from QtCore import QMutex, QMutexLocker, QObject

from Settings import Settings

class SettingsProxy(QObject):

    def __init__(self, qsettings):
        super(SettingsProxy, self).__init__()
        self.setObjectName('SettingsProxy')

        self._qsettings = qsettings
        self._mutex = QMutex(QMutex.Recursive)

    def get_settings(self, group):
        return Settings(self, group)

    def all_keys(self, group):
        locker = QMutexLocker(self._mutex) #@UnusedVariable
        self._qsettings.beginGroup(group)
        keys = self._qsettings.allKeys()
        self._qsettings.endGroup()
        return keys

#    def begin_read_array(self, group):

#    def begin_write_array(self, group):

    def child_groups(self, group):
        locker = QMutexLocker(self._mutex) #@UnusedVariable
        self._qsettings.beginGroup(group)
        groups = self._qsettings.childGroups()
        self._qsettings.endGroup()
        return groups

    def child_keys(self, group):
        locker = QMutexLocker(self._mutex) #@UnusedVariable
        self._qsettings.beginGroup(group)
        keys = self._qsettings.childKeys()
        self._qsettings.endGroup()
        return keys

    def contains(self, group, key):
        locker = QMutexLocker(self._mutex) #@UnusedVariable
        self._qsettings.beginGroup(group)
        key_exists = self._qsettings.contains(key)
        self._qsettings.endGroup()
        return key_exists

#    def end_array(self):

    def remove(self, group, key):
        locker = QMutexLocker(self._mutex) #@UnusedVariable
        self._qsettings.beginGroup(group)
        self._qsettings.remove(key)
        self._qsettings.endGroup()

#    def set_array_index(self, i):

    def set_value(self, group, key, value):
        locker = QMutexLocker(self._mutex) #@UnusedVariable
        self._qsettings.beginGroup(group)
        self._qsettings.setValue(key, value)
        self._qsettings.endGroup()

    def value(self, group, key, defaultValue=None):
        locker = QMutexLocker(self._mutex) #@UnusedVariable
        self._qsettings.beginGroup(group)
        v = self._qsettings.value(key, defaultValue)
        self._qsettings.endGroup()
        return v
