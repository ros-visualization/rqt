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
        self._main_window = main_window
        self._plugin_instance_id = plugin_instance_id
        self._hide_close_button_flag = hide_close_button
        self._plugin_instance = None
        self._dock_widgets = []
        self.hide()


    # pointer to QDockWidget must be used for PySide to work (at least with 1.0.1)
    @Slot(int, 'QDockWidget*')
    def addDockWidget(self, area, dock_widget):
        # generate unique object name for this dock_widget
        unique_object_name = os.path.join(self._plugin_instance_id, dock_widget.objectName())
        # find and remove possible remaining dock_widget with this object name
        old_dock_widget = self._main_window.findChild(QDockWidget, unique_object_name)
        if old_dock_widget is not None:
            qWarning('MainWindowInterface.addDockWidget() duplicate object name "%s", removing old dock widget!')
            self._main_window.removeDockWidget(old_dock_widget)
        # rename dock_widget object
        dock_widget.setObjectName(unique_object_name)
        # add new dock_widget, area must be casted for PySide to work (at least with 1.0.1)
        self._main_window.addDockWidget(Qt.DockWidgetArea(area), dock_widget)
        self._dock_widgets.append(dock_widget)
        self._update_title_bar(dock_widget)


    # pointer to QDockWidget must be used for PySide to work (at least with 1.0.1)
    @Slot('QDockWidget*')
    def removeDockWidget(self, dock_widget):
        self._main_window.removeDockWidget(dock_widget)
        self._dock_widgets.remove(dock_widget)


    # pointer to QObject must be used for PySide to work (at least with 1.0.1)
    @Slot('QObject*')
    def set_plugin_instance(self, plugin_instance):
        self._plugin_instance = plugin_instance
        for dock_widget in self._dock_widgets:
            self._update_title_bar(dock_widget)


    def _update_title_bar(self, dock_widget):
        title_bar = dock_widget.titleBarWidget()
        if title_bar is None:
            title_bar = DockWidgetTitleBar(dock_widget, self._hide_close_button_flag)
            dock_widget.setTitleBarWidget(title_bar)

            # connect extra buttons
            title_bar.connect_button('help', self._help_request)
            title_bar.connect_button('reload', self._reload_request)

            # connect settings button to plugin instance
            if hasattr(self._plugin_instance, 'settings_request'):
                title_bar.connect_button('settings', getattr(self._plugin_instance, 'settings_request'))
                title_bar.show_button('settings')
            else:
                title_bar.hide_button('settings')


    def _help_request(self):
        self.plugin_help_signal.emit(self._plugin_instance_id)


    def _reload_request(self):
        self.reload_plugin_instance_signal.emit(self._plugin_instance_id)


    def save_settings(self, perspective_settings):
        self._call_method_on_all_title_bars('save_settings', perspective_settings)


    def restore_settings(self, perspective_settings):
        self._call_method_on_all_title_bars('restore_settings', perspective_settings)


    def _call_method_on_all_title_bars(self, method_name, perspective_settings):
        settings = perspective_settings.get_settings('mainwindowinterface')
        for dock_widget in self._dock_widgets:
            name = 'title_bar__' + dock_widget.objectName().replace('/', '_')
            perspective = settings.get_settings(name)
            title_bar = dock_widget.titleBarWidget()
            if hasattr(title_bar, method_name):
                method = getattr(title_bar, method_name)
                try:
                    method(perspective)
                except Exception:
                    qCritical('MainWindowInterface._call_method_on_all_title_bars(%s) call on DockWidgetTitleBar failed:\n%s' % (method_name, traceback.format_exc()))
