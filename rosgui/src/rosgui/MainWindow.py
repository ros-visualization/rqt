import QtBindingHelper #@UnusedImport
from QtCore import qDebug, Signal
from QtGui import QMainWindow

from Settings import Settings

class MainWindow(QMainWindow):

    save_settings_signal = Signal(Settings, Settings)

    def __init__(self):
        super(MainWindow, self).__init__()
        self.setObjectName('MainWindow')

        self.global_settings_ = None
        self.perspective_settings_ = None
        self.settings_ = None

    def closeEvent(self, event):
        qDebug('MainWindow.closeEvent()')
        self._save_geometry_to_perspective()
        self._save_state_to_perspective()
        self.save_settings_signal.emit(self.global_settings_, self.perspective_settings_)
        event.accept()

    def save_settings(self, global_settings, perspective_settings):
        qDebug('MainWindow.save_settings()')
        self.global_settings_ = global_settings
        self.perspective_settings_ = perspective_settings
        self.settings_ = self.perspective_settings_.get_settings('mainwindow')
        # store current setup to current settings
        self._save_geometry_to_perspective()
        self._save_state_to_perspective()

    def restore_settings(self, global_settings, perspective_settings):
        qDebug('MainWindow.restore_settings()')
        # remember new settings
        self.global_settings_ = global_settings
        self.perspective_settings_ = perspective_settings
        self.settings_ = self.perspective_settings_.get_settings('mainwindow')
        # only restore geometry, restoring state is triggered after PluginManager has been updated
        self._restore_geometry_from_perspective()

    def save_setup(self):
        qDebug('MainWindow.save_setup()')
        # store current setup to current settings
        self._save_geometry_to_perspective()
        self._save_state_to_perspective()

    def restore_state(self):
        qDebug('MainWindow.restore_state()')
        # restore state from settings
        self._restore_state_from_perspective()

    def perspective_changed(self, name):
        self.setWindowTitle('%s - RosGui' % str(name))

    def _save_geometry_to_perspective(self):
        if self.settings_ is not None:
            self.settings_.set_value('geometry', self.saveGeometry())

    def _restore_geometry_from_perspective(self):
        if self.settings_.contains('geometry'):
            restored = self.restoreGeometry(self.settings_.value('geometry'))

    def _save_state_to_perspective(self):
        if self.settings_ is not None:
            self.settings_.set_value('state', self.saveState())

    def _restore_state_from_perspective(self):
        if self.settings_.contains('state'):
            restored = self.restoreState(self.settings_.value('state'))
