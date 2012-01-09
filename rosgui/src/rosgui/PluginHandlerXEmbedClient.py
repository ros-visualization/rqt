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

import os
import traceback

from dbus import Interface
from dbus.connection import Connection
from PluginHandlerDirect import PluginHandlerDirect
from QtCore import qCritical, qDebug, Slot
from QtGui import QVBoxLayout, QX11EmbedWidget
from Settings import Settings

class PluginHandlerXEmbedClient(PluginHandlerDirect):

    """
    Client part of the `PluginHandlerXEmbed`.
    It utilizes the `PluginHandlerDBusService` of the `PluginHandlerXEmbedContainer` through a peer-to-peer DBus connection.
    """

    def __init__(self, main_window, instance_id, application_context, container_manager, dbus_object_path):
        super(PluginHandlerXEmbedClient, self).__init__(main_window, instance_id, application_context, container_manager)
        self.setObjectName('PluginHandlerXEmbedClient')
        self._dbus_object_path = dbus_object_path
        self._remote_container = None
        self._remote_global_settings = None
        self._remote_perspective_settings = None
        self._embed_widgets = {}


    def _load(self):
        conn = Connection(self._application_context.options.embed_plugin_address)
        proxy = conn.get_object(None, self._dbus_object_path)
        self._remote_container = Interface(proxy, 'org.ros.rosgui.PluginHandlerContainer')
        self._remote_container.connect_to_signal('shutdown_plugin', self._shutdown_plugin)
        self._remote_container.connect_to_signal('save_settings', self._save_settings_from_remote)
        self._remote_container.connect_to_signal('restore_settings', self._restore_settings_from_remote)
        self._remote_container.connect_to_signal('trigger_configuration', self._trigger_configuration)

        proxy = conn.get_object(None, self._dbus_object_path + '/global')
        self._remote_global_settings = Interface(proxy, 'org.ros.rosgui.Settings')
        proxy = conn.get_object(None, self._dbus_object_path + '/perspective')
        self._remote_perspective_settings = Interface(proxy, 'org.ros.rosgui.Settings')

        super(PluginHandlerXEmbedClient, self)._load()

    def _emit_load_completed(self, exception=None):
        # signal failed loading before emitting signal, as it might not be possible afterwards
        if exception is not None:
            self._remote_container.load_completed(False, False)
        super(PluginHandlerXEmbedClient, self)._emit_load_completed(exception)
        # signal successful loading after emitting signal, for better message order
        if exception is None:
            self._remote_container.load_completed(True, self._plugin_has_configuration)


    def shutdown_plugin(self, callback):
        # this method should never be called for embedded clients
        assert(False)

    def emit_shutdown_plugin_completed(self):
        self._remote_container.shutdown_plugin_completed()


    def save_settings(self, global_settings, perspective_settings, callback=None):
        # this method should never be called for embedded clients
        assert(False)

    def _save_settings_from_remote(self):
        qDebug('PluginHandlerXEmbedClient._save_settings_from_remote()')
        try:
            global_settings = Settings(self._remote_global_settings, '')
            perspective_settings = Settings(self._remote_perspective_settings, '')
            self._save_settings(global_settings, perspective_settings)
        except Exception:
            qCritical('PluginHandlerXEmbedClient._save_settings_from_remote() plugin "%s" raised an exception:\n%s' % (str(self._instance_id), traceback.format_exc()))
            self.emit_save_settings_completed()

    def emit_save_settings_completed(self):
        self._remote_container.save_settings_completed()


    def restore_settings(self, global_settings, perspective_settings, callback=None):
        # this method should never be called for embedded clients
        assert(False)

    def _restore_settings_from_remote(self):
        qDebug('PluginHandlerXEmbedClient._restore_settings_from_remote()')
        try:
            global_settings = Settings(self._remote_global_settings, '')
            perspective_settings = Settings(self._remote_perspective_settings, '')
            self._restore_settings(global_settings, perspective_settings)
        except Exception:
            qCritical('PluginHandlerXEmbedClient._restore_settings_from_remote() plugin "%s" raised an exception:\n%s' % (str(self._instance_id), traceback.format_exc()))
            self.emit_restore_settings_completed()

    def emit_restore_settings_completed(self):
        self._remote_container.restore_settings_completed()


    # pointer to QWidget must be used for PySide to work (at least with 1.0.1)
    @Slot('QWidget*')
    def add_widget(self, widget):
        embed_widget = QX11EmbedWidget()
        layout = QVBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(widget)
        embed_widget.setLayout(layout)

        # close embed widget when container is closed
        # TODO nceessary?
        #embed_widget.containerClosed.connect(embed_widget.close)

        embed_container_window_id = self._remote_container.embed_widget(os.getpid(), widget.objectName())
        embed_widget.embedInto(embed_container_window_id)

        self._embed_widgets[widget] = embed_widget
        self.update_widget_title(widget)

        embed_widget.show()

    def _update_widget_title(self, widget, title):
        self._remote_container.update_embedded_widget_title(widget.objectName(), title)

    # pointer to QWidget must be used for PySide to work (at least with 1.0.1)
    @Slot('QWidget*')
    def remove_widget(self, widget):
        self._remote_container.unembed_widget(widget.objectName())
        # do not delete the widget, only the embed widget
        widget.setParent(None)
        del self._embed_widgets[widget]
        # triggering close after last widget is closed is handled by the container

    def _emit_close_plugin(self):
        self._remote_container.close_plugin()
