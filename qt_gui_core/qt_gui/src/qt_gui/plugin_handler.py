# Copyright (c) 2011, Dirk Thomas, Dorian Scholz, TU Darmstadt
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the TU Darmstadt nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import traceback

from . import qt_binding_helper  # @UnusedImport
from QtCore import qCritical, qDebug, QObject, Qt, qWarning, Signal, Slot
from QtGui import QDockWidget

from .dock_widget import DockWidget
from .dock_widget_title_bar import DockWidgetTitleBar
from .window_title_changed_signaler import WindowTitleChangedSignaler


class PluginHandler(QObject):

    """
    Base class for the bidirectional exchange between the framework and one `Plugin` instance.
    It utilizes a `PluginProvider` to load/unload the plugin and provides callbacks for the `PluginContext`.
    """

    close_signal = Signal(str)
    reload_signal = Signal(str)
    help_signal = Signal(str)

    def __init__(self, main_window, instance_id, application_context, container_manager):
        super(PluginHandler, self).__init__()
        self.setObjectName('PluginHandler')

        self._main_window = main_window
        self._instance_id = instance_id
        self._application_context = application_context
        self._container_manager = container_manager

        self._plugin_provider = None
        self.__callback = None
        self.__instance_settings = None

        self._plugin_has_configuration = False

        # mapping of added widgets to their parent dock widget and WindowTitleChangedSignaler
        self._widgets = {}

    def instance_id(self):
        return self._instance_id

    def load(self, plugin_provider, callback=None):
        """
        Load plugin.
        Completion is signaled asynchronously if a callback is passed.
        """
        self._plugin_provider = plugin_provider
        self.__callback = callback
        try:
            self._load()
        except Exception as e:
            self._emit_load_completed(e)

    def _load(self):
        raise NotImplementedError

    def _update_title_bars(self):
        if self._plugin_has_configuration:
            for dock_widget, _ in self._widgets.values():
                title_bar = dock_widget.titleBarWidget()
                title_bar.show_button('configuration')

    def _emit_load_completed(self, exception=None):
        if exception is not None:
            # garbage already registered widgets
            for widget in self._widgets.keys():
                self.remove_widget(widget)
                self._delete_widget(widget)
        if self.__callback is not None:
            callback = self.__callback
            self.__callback = None
            callback(self, exception)
        elif exception is not None:
            qCritical('PluginHandler.load() failed%s' % (':\n%s' % str(exception) if exception != True else ''))

    def shutdown_plugin(self, callback):
        """
        Shutdown plugin (`Plugin.shutdown_settings()`) and remove all added widgets.
        Completion is signaled asynchronously if a callback is passed.
        """
        self.__callback = callback
        try:
            self._shutdown_plugin()
        except Exception:
            qCritical('PluginHandler.shutdown_plugin() plugin "%s" raised an exception:\n%s' % (str(self._instance_id), traceback.format_exc()))
            self.emit_shutdown_plugin_completed()

    def _shutdown_plugin(self):
        raise NotImplementedError

    def emit_shutdown_plugin_completed(self):
        for widget in self._widgets.keys():
            self.remove_widget(widget)
            self._delete_widget(widget)
        if self.__callback is not None:
            callback = self.__callback
            self.__callback = None
            callback(self._instance_id)

    def _delete_widget(self, widget):
        del widget

    def unload(self, callback=None):
        """
        Unload plugin.
        Completion is signaled asynchronously if a callback is passed.
        """
        self.__callback = callback
        try:
            self._unload()
        except Exception:
            qCritical('PluginHandler.unload() plugin "%s" raised an exception:\n%s' % (str(self._instance_id), traceback.format_exc()))
            self._emit_unload_completed()

    def _unload(self):
        raise NotImplementedError

    def _emit_unload_completed(self):
        if self.__callback is not None:
            callback = self.__callback
            self.__callback = None
            callback(self._instance_id)

    def save_settings(self, plugin_settings, instance_settings, callback=None):
        """
        Save settings of the plugin (`Plugin.save_settings()`) and all dock widget title bars.
        Completion is signaled asynchronously if a callback is passed.
        """
        qDebug('PluginHandler.save_settings()')
        self.__instance_settings = instance_settings
        self.__callback = callback
        try:
            self._save_settings(plugin_settings, instance_settings)
        except Exception:
            qCritical('PluginHandler.save_settings() plugin "%s" raised an exception:\n%s' % (str(self._instance_id), traceback.format_exc()))
            self.emit_save_settings_completed()

    def _save_settings(self, plugin_settings, instance_settings):
        raise NotImplementedError

    def emit_save_settings_completed(self):
        qDebug('PluginHandler.emit_save_settings_completed()')
        self._call_method_on_all_dock_widgets('save_settings', self.__instance_settings)
        self.__instance_settings = None
        if self.__callback is not None:
            callback = self.__callback
            self.__callback = None
            callback(self._instance_id)

    def _call_method_on_all_dock_widgets(self, method_name, instance_settings):
        for dock_widget, _ in self._widgets.values():
            name = 'dock_widget' + dock_widget.objectName().replace(self._instance_id.tidy_str(), '', 1)
            settings = instance_settings.get_settings(name)
            method = getattr(dock_widget, method_name)
            try:
                method(settings)
            except Exception:
                qCritical('PluginHandler._call_method_on_all_dock_widgets(%s) failed:\n%s' % (method_name, traceback.format_exc()))

    def restore_settings(self, plugin_settings, instance_settings, callback=None):
        """
        Restore settings of the plugin (`Plugin.restore_settings()`) and all dock widget title bars.
        Completion is signaled asynchronously if a callback is passed.
        """
        qDebug('PluginHandler.restore_settings()')
        self.__instance_settings = instance_settings
        self.__callback = callback
        try:
            self._restore_settings(plugin_settings, instance_settings)
        except Exception:
            qCritical('PluginHandler.restore_settings() plugin "%s" raised an exception:\n%s' % (str(self._instance_id), traceback.format_exc()))
            self.emit_restore_settings_completed()

    def _restore_settings(self, plugin_settings, instance_settings):
        raise NotImplementedError

    def emit_restore_settings_completed(self):
        qDebug('PluginHandler.emit_restore_settings_completed()')
        # call after plugin has restored settings as it may spawn additional dock widgets
        self._call_method_on_all_dock_widgets('restore_settings', self.__instance_settings)
        self.__instance_settings = None
        if self.__callback is not None:
            callback = self.__callback
            self.__callback = None
            callback(self._instance_id)

    def _create_dock_widget(self):
        dock_widget = DockWidget(self._container_manager)
        if self._application_context.options.lock_perspective is not None or self._application_context.options.standalone_plugin is not None:
            # plugins are not closable when perspective is locked or plugins is running standalone
            features = dock_widget.features()
            dock_widget.setFeatures(features ^ QDockWidget.DockWidgetClosable)
        self._update_title_bar(dock_widget)
        return dock_widget

    def _update_title_bar(self, dock_widget, hide_help=False, hide_reload=False):
        title_bar = dock_widget.titleBarWidget()
        if title_bar is None:
            title_bar = DockWidgetTitleBar(dock_widget)
            dock_widget.setTitleBarWidget(title_bar)

            # connect extra buttons
            title_bar.connect_close_button(self._close_dock_widget)
            title_bar.connect_button('help', self._emit_help_signal)
            if hide_help:
                title_bar.hide_button('help')
            title_bar.connect_button('reload', self._emit_reload_signal)
            if hide_reload:
                title_bar.hide_button('reload')
            title_bar.connect_button('configuration', self._trigger_configuration)
            # hide configuration button until existence of feature has been confirmed
            title_bar.hide_button('configuration')

    def _close_dock_widget(self, dock_widget):
        widget = [key for key, value in self._widgets.iteritems() if value[0] == dock_widget][0]
        self.remove_widget(widget)

    def _emit_help_signal(self):
        self.help_signal.emit(str(self._instance_id))

    def _emit_reload_signal(self):
        self.reload_signal.emit(str(self._instance_id))

    def _trigger_configuration(self):
        self._plugin.trigger_configuration()

    def _add_dock_widget(self, dock_widget, widget):
        self._add_dock_widget_to_main_window(dock_widget)
        signaler = WindowTitleChangedSignaler(widget, widget)
        signaler.window_title_changed_signal.connect(self._on_widget_title_changed)
        self._widgets[widget] = [dock_widget, signaler]
        # trigger to update initial window title
        signaler.window_title_changed_signal.emit(widget)

    def _add_dock_widget_to_main_window(self, dock_widget):
        if self._main_window is not None:
            # find and remove possible remaining dock_widget with this object name
            old_dock_widget = self._main_window.findChild(DockWidget, dock_widget.objectName())
            if old_dock_widget is not None:
                qWarning('PluginHandler._add_dock_widget_to_main_window() duplicate object name "%s", removing old dock widget!' % dock_widget.objectName())
                self._main_window.removeDockWidget(old_dock_widget)
            self._main_window.addDockWidget(Qt.BottomDockWidgetArea, dock_widget)

    def _on_widget_title_changed(self, widget):
        dock_widget, _ = self._widgets[widget]
        dock_widget.setWindowTitle(widget.windowTitle())

    def _update_widget_title(self, widget, title):
        dock_widget, _ = self._widgets[widget]
        dock_widget.setWindowTitle(title)

    # pointer to QWidget must be used for PySide to work (at least with 1.0.1)
    @Slot('QWidget*')
    def remove_widget(self, widget):
        dock_widget, signaler = self._widgets[widget]
        if signaler is not None:
            signaler.window_title_changed_signal.disconnect(self._on_widget_title_changed)
        self._remove_dock_widget_from_parent(dock_widget)
        # do not delete the widget, only the dock widget
        widget.setParent(None)
        del self._widgets[widget]
        # close plugin when last widget is removed
        if len(self._widgets) == 0:
            self._emit_close_plugin()

    def _remove_dock_widget_from_parent(self, dock_widget):
        if self._main_window is not None:
            dock_widget.parent().removeDockWidget(dock_widget)
            dock_widget.setParent(None)
            dock_widget.deleteLater()

    def _emit_close_plugin(self):
        self.close_signal.emit(str(self._instance_id))
