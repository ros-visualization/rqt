import QtBindingHelper
from QtCore import QMutex, QMutexLocker, QObject

from Settings import Settings

class SettingsProxy(QObject):

    def __init__(self, qsettings):
        QObject.__init__(self)
        self.setObjectName('SettingsProxy')

        self.qsettings_ = qsettings
        self.mutex_ = QMutex(QMutex.Recursive)

    def get_settings(self, group):
        return Settings(self, group)

    def all_keys(self, group):
        locker = QMutexLocker(self.mutex_) #@UnusedVariable
        self.qsettings_.beginGroup(group)
        keys = self.qsettings_.allKeys()
        self.qsettings_.endGroup()
        return keys

#    def begin_read_array(self, group):

#    def begin_write_array(self, group):

    def child_groups(self, group):
        locker = QMutexLocker(self.mutex_) #@UnusedVariable
        self.qsettings_.beginGroup(group)
        groups = self.qsettings_.childGroups()
        self.qsettings_.endGroup()
        return groups

    def child_keys(self, group):
        locker = QMutexLocker(self.mutex_) #@UnusedVariable
        self.qsettings_.beginGroup(group)
        keys = self.qsettings_.childKeys()
        self.qsettings_.endGroup()
        return keys

    def contains(self, group, key):
        locker = QMutexLocker(self.mutex_) #@UnusedVariable
        self.qsettings_.beginGroup(group)
        key_exists = self.qsettings_.contains(key)
        self.qsettings_.endGroup()
        return key_exists

#    def end_array(self):

    def remove(self, group, key):
        locker = QMutexLocker(self.mutex_) #@UnusedVariable
        self.qsettings_.beginGroup(group)
        self.qsettings_.remove(key)
        self.qsettings_.endGroup()

#    def set_array_index(self, i):

    def set_value(self, group, key, value):
        locker = QMutexLocker(self.mutex_) #@UnusedVariable
        self.qsettings_.beginGroup(group)
        self.qsettings_.setValue(key, value)
        self.qsettings_.endGroup()

    def value(self, group, key, defaultValue):
        locker = QMutexLocker(self.mutex_) #@UnusedVariable
        self.qsettings_.beginGroup(group)
        v = self.qsettings_.value(key, defaultValue)
        self.qsettings_.endGroup()
        return v
