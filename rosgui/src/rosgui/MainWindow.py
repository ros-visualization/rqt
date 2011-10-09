import QtBindingHelper
from QtCore import qDebug, Signal
from QtGui import QMainWindow

from Settings import Settings

class MainWindow(QMainWindow):

    save_settings_signal = Signal(Settings, Settings)

    def __init__(self):
        QMainWindow.__init__(self)
        self.setObjectName('MainWindow')

        self.global_settings_ = None
        self.perspective_settings_ = None
        self.settings_ = None

    def closeEvent(self, event):
        qDebug('MainWindow.closeEvent()')
        self.__save_geometry_to_perspective()
        self.__save_state_to_perspective()
        self.save_settings_signal.emit(self.global_settings_, self.perspective_settings_)
        event.accept()

    def save_setup(self):
        qDebug('MainWindow.save_setup()')
        # store current setup to current settings
        self.__save_geometry_to_perspective()
        self.__save_state_to_perspective()

    def save_settings(self, global_settings, perspective_settings):
        qDebug('MainWindow.save_settings()')
        self.global_settings_ = global_settings
        self.perspective_settings_ = perspective_settings
        self.settings_ = self.perspective_settings_.get_settings('mainwindow')
        self.save_setup()

    def restore_settings(self, global_settings, perspective_settings):
        qDebug('MainWindow.restore_settings()')
        # remember new settings - delay restore after PluginManager has been updated
        self.global_settings_ = global_settings
        self.perspective_settings_ = perspective_settings
        self.settings_ = self.perspective_settings_.get_settings('mainwindow')
        self.__restore_geometry_from_perspective()

    def restore_state(self):
        qDebug('MainWindow.restore_state()')
        # restore state from settings
        self.__restore_state_from_perspective()

    def perspective_changed(self, name):
        self.setWindowTitle('%s - RosGui' % str(name))

    def __save_geometry_to_perspective(self):
        if self.settings_ is not None:
            qDebug('MainWindow.__save_geometry_to_perspective()')
            self.settings_.set_value('geometry', self.saveGeometry())

    def __restore_geometry_from_perspective(self):
        if self.settings_.contains('geometry'):
            restored = self.restoreGeometry(self.settings_.value('geometry'))
            qDebug('MainWindow.__restore_geometry_from_perspective() %s' % ('successful' if restored else 'failed'))

    def __save_state_to_perspective(self):
        if self.settings_ is not None:
            qDebug('MainWindow.__save_state_to_perspective()')
            self.settings_.set_value('state', self.saveState())

    def __restore_state_from_perspective(self):
        if self.settings_.contains('state'):
            restored = self.restoreState(self.settings_.value('state'))
            qDebug('MainWindow.__restore_state_from_perspective() %s' % ('successful' if restored else 'failed'))
