import QtBindingHelper #@UnusedImport
from QtCore import qDebug, Signal
from QtGui import QMainWindow

from Settings import Settings

class MainWindow(QMainWindow):

    save_settings_signal = Signal(Settings, Settings)

    def __init__(self):
        super(MainWindow, self).__init__()
        self.setObjectName('MainWindow')

        self._global_settings = None
        self._perspective_settings = None
        self._settings = None

    def closeEvent(self, event):
        qDebug('MainWindow.closeEvent()')
        self._save_geometry_to_perspective()
        self._save_state_to_perspective()
        self.save_settings_signal.emit(self._global_settings, self._perspective_settings)
        event.accept()

    def save_settings(self, global_settings, perspective_settings):
        qDebug('MainWindow.save_settings()')
        self._global_settings = global_settings
        self._perspective_settings = perspective_settings
        self._settings = self._perspective_settings.get_settings('mainwindow')
        # store current setup to current _settings
        self._save_geometry_to_perspective()
        self._save_state_to_perspective()

    def restore_settings(self, global_settings, perspective_settings):
        qDebug('MainWindow.restore_settings()')
        # remember new _settings
        self._global_settings = global_settings
        self._perspective_settings = perspective_settings
        self._settings = self._perspective_settings.get_settings('mainwindow')
        # only restore geometry, restoring state is triggered after PluginManager has been updated
        self._restore_geometry_from_perspective()

    def save_setup(self):
        qDebug('MainWindow.save_setup()')
        # store current setup to current _settings
        self._save_geometry_to_perspective()
        self._save_state_to_perspective()

    def restore_state(self):
        qDebug('MainWindow.restore_state()')
        # restore state from _settings
        self._restore_state_from_perspective()

    def perspective_changed(self, name):
        self.setWindowTitle('%s - RosGui' % str(name))

    def _save_geometry_to_perspective(self):
        if self._settings is not None:
            self._settings.set_value('geometry', self.saveGeometry())

    def _restore_geometry_from_perspective(self):
        if self._settings.contains('geometry'):
            self.restoreGeometry(self._settings.value('geometry'))

    def _save_state_to_perspective(self):
        if self._settings is not None:
            self._settings.set_value('state', self.saveState())

    def _restore_state_from_perspective(self):
        if self._settings.contains('state'):
            self.restoreState(self._settings.value('state'))
