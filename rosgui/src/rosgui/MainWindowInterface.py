import os, traceback

import QtBindingHelper #@UnusedImport
from QtCore import qCritical, Qt, qWarning, Signal, Slot
from QtGui import QDockWidget, QWidget

from DockWidgetTitleBar import DockWidgetTitleBar

class MainWindowInterface(QWidget):

    plugin_help_signal = Signal(str)
    reload_plugin_instance_signal = Signal(str)

    def __init__(self, main_window, plugin_instance_id, hide_close_button=False):
        super(MainWindowInterface, self).__init__(main_window)
        self.main_window_ = main_window
        self.plugin_instance_id_ = plugin_instance_id
        self.hide_close_button = hide_close_button
        self.plugin_instance_ = None
        self.dock_widgets_ = []


    # pointer to QDockWidget must be used for PySide to work (at least with 1.0.1)
    @Slot(int, 'QDockWidget*')
    def addDockWidget(self, area, dock_widget):
        # generate unique object name for this dock_widget
        unique_object_name = os.path.join(self.plugin_instance_id_, dock_widget.objectName())
        # find and remove possible remaining dock_widget with this object name
        old_dock_widget = self.main_window_.findChild(QDockWidget, unique_object_name)
        if old_dock_widget is not None:
            qWarning('MainWindowInterface.addDockWidget() duplicate object name "%s", removing old dock widget!')
            self.main_window_.removeDockWidget(old_dock_widget)
        # rename dock_widget object
        dock_widget.setObjectName(unique_object_name)
        # add new dock_widget, area must be casted for PySide to work (at least with 1.0.1)
        self.main_window_.addDockWidget(Qt.DockWidgetArea(area), dock_widget)
        self.dock_widgets_.append(dock_widget)
        self._update_title_bar(dock_widget)


    # pointer to QDockWidget must be used for PySide to work (at least with 1.0.1)
    @Slot('QDockWidget*')
    def removeDockWidget(self, dock_widget):
        self.main_window_.removeDockWidget(dock_widget)
        self.dock_widgets_.remove(dock_widget)


    # pointer to QObject must be used for PySide to work (at least with 1.0.1)
    @Slot('QObject*')
    def set_plugin_instance(self, plugin_instance):
        self.plugin_instance_ = plugin_instance
        for dock_widget in self.dock_widgets_:
            self._update_title_bar(dock_widget)


    def _update_title_bar(self, dock_widget):
        title_bar = dock_widget.titleBarWidget()
        if title_bar is None:
            title_bar = DockWidgetTitleBar(dock_widget, self.hide_close_button)
            dock_widget.setTitleBarWidget(title_bar)

            # connect extra buttons
            title_bar.connect_button('help', self._help_request)
            title_bar.connect_button('reload', self._reload_request)

        # connect settings button to plugin instance
        if hasattr(self.plugin_instance_, 'settings_request'):
            title_bar.connect_button('settings', getattr(self.plugin_instance_, 'settings_request'))
            title_bar.show_button('settings')
        else:
            title_bar.hide_button('settings')


    def _help_request(self):
        self.plugin_help_signal.emit(self.plugin_instance_id_)


    def _reload_request(self):
        self.reload_plugin_instance_signal.emit(self.plugin_instance_id_)


    def save_settings(self, perspective_settings):
        self._call_method_on_all_title_bars('save_settings', perspective_settings)


    def restore_settings(self, perspective_settings):
        self._call_method_on_all_title_bars('restore_settings', perspective_settings)


    def _call_method_on_all_title_bars(self, method_name, perspective_settings):
        settings = perspective_settings.get_settings('mainwindowinterface')
        for dock_widget in self.dock_widgets_:
            name = 'title_bar__' + dock_widget.objectName().replace('/', '_')
            perspective = settings.get_settings(name)
            title_bar = dock_widget.titleBarWidget()
            if hasattr(title_bar, method_name):
                method = getattr(title_bar, method_name)
                try:
                    method(perspective)
                except Exception:
                    qCritical('MainWindowInterface._call_method_on_all_title_bars(%s) call on DockWidgetTitleBar failed:\n%s' % (method_name, traceback.format_exc()))
