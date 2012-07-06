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

import sys

from dbus.server import Server
from .plugin_handler import PluginHandler
from .plugin_handler_dbus_service import PluginHandlerDBusService
from .qt_binding_helper import QT_BINDING
from QtCore import qDebug, QProcess, qWarning
from QtGui import QX11EmbedContainer
from .main import Main
from .settings_proxy_dbus_service import SettingsProxyDBusService


class PluginHandlerXEmbedContainer(PluginHandler):

    """
    Server part of the `PluginHandlerXEmbed`.
    It starts the plugin in a subprocess and provides the `PluginHandlerDBusService` through a peer-to-peer DBus connection.
    """

    _serial_number = 0

    def __init__(self, main_window, instance_id, application_context, container_manager, dbus_object_path):
        super(PluginHandlerXEmbedContainer, self).__init__(main_window, instance_id, application_context, container_manager)
        self.setObjectName('PluginHandlerXEmbedContainer')

        self._dbus_object_path = dbus_object_path
        self._dbus_server = None
        self._dbus_container_service = None
        self._dbus_plugin_settings_service = None
        self._dbus_instance_settings_service = None

        self._process = None
        self._pid = None
        # mapping of widget object name to their embed container
        self._embed_containers = {}

        self._objects_and_paths = []

    def _load(self):
        self._dbus_server = Server('tcp:bind=*')
        self._dbus_server.on_connection_added.append(self._add_dbus_connection)
        self._dbus_container_service = PluginHandlerDBusService(self, self._dbus_object_path)
        self._dbus_plugin_settings_service = SettingsProxyDBusService(self._dbus_object_path + '/plugin')
        self._dbus_instance_settings_service = SettingsProxyDBusService(self._dbus_object_path + '/instance')

        self._process = QProcess(self)
        self._process.setProcessChannelMode(QProcess.SeparateChannels)
        self._process.readyReadStandardOutput.connect(self._print_process_output)
        self._process.readyReadStandardError.connect(self._print_process_error)
        self._process.finished.connect(self._emit_close_plugin)
        # start python with unbuffered stdout/stderr so that the order of the output is retained
        cmd = sys.executable + ' -u'
        cmd += ' %s' % Main.main_filename
        cmd += ' --qt-binding=%s' % QT_BINDING
        cmd += ' --embed-plugin=%s --embed-plugin-serial=%s --embed-plugin-address=%s' % (self.instance_id().plugin_id, self.instance_id().serial_number, self._dbus_server.address)
        #qDebug('PluginHandlerXEmbedContainer._load() starting command: %s' % cmd)
        self._process.start(cmd)
        started = self._process.waitForStarted(3000)
        if not started:
            self._dbus_container_service.remove_from_connection()
            self._dbus_plugin_settings_service.remove_from_connection()
            self._dbus_instance_settings_service.remove_from_connection()
            raise RuntimeError('PluginHandlerXEmbedContainer._load() could not start subprocess in reasonable time')
        # QProcess.pid() has been added to PySide in 1.0.5
        if hasattr(self._process, 'pid'):
            self._pid = self._process.pid()
        else:
            # use serial number as a replacement for pid if not available
            self.__class__._serial_number = self._serial_number + 1
            self._pid = self._serial_number

        qDebug('PluginHandlerXEmbedContainer._load() started subprocess (#%s) for plugin "%s"' % (self._pid, str(self._instance_id)))
        # self._emit_load_completed is called asynchronous when client signals finished loading via dbus

    def _add_dbus_connection(self, conn):
        self._dbus_container_service.add_to_connection(conn, self._dbus_object_path)
        self._dbus_plugin_settings_service.add_to_connection(conn, self._dbus_object_path + '/plugin')
        self._dbus_instance_settings_service.add_to_connection(conn, self._dbus_object_path + '/instance')

    def _print_process_output(self):
        self._print_process(self._process.readAllStandardOutput(), qDebug)

    def _print_process_error(self):
        self._print_process(self._process.readAllStandardError(), qWarning)

    def _print_process(self, data, method):
        # indent process output and prefix it with the pid
        lines = str(data).split('\n')
        if lines[-1] == '':
            lines.pop()
        for line in lines:
            method('    %d %s' % (self._pid, line))

    def load_completed(self, loaded, has_configuration):
        # TODO timer to detect no response
        exception = None if loaded else True
        self._plugin_has_configuration = has_configuration
        self._update_title_bars()
        self._emit_load_completed(exception)

    def _shutdown_plugin(self):
        qDebug('PluginHandlerXEmbedContainer._shutdown_plugin()')
        self._process.finished.disconnect(self._emit_close_plugin)
        self._dbus_container_service.shutdown_plugin()

    def emit_shutdown_plugin_completed(self):
        self._dbus_container_service.remove_from_connection()
        self._dbus_plugin_settings_service.remove_from_connection()
        self._dbus_instance_settings_service.remove_from_connection()

        self._process.close()
        self._process.waitForFinished(5000)
        if self._process.state() != QProcess.NotRunning:
            self._process.kill()
        self._process = None

        super(PluginHandlerXEmbedContainer, self).emit_shutdown_plugin_completed()

    def _unload(self):
        qDebug('PluginHandlerXEmbedContainer._unload()')
        self._emit_unload_completed()

    def _save_settings(self, plugin_settings, instance_settings):
        qDebug('PluginHandlerXEmbedContainer._save_settings()')
        self._dbus_plugin_settings_service.set_settings(plugin_settings)
        self._dbus_instance_settings_service.set_settings(instance_settings)
        self._dbus_container_service.save_settings()

    def emit_save_settings_completed(self):
        self._dbus_plugin_settings_service.set_settings(None)
        self._dbus_instance_settings_service.set_settings(None)
        super(PluginHandlerXEmbedContainer, self).emit_save_settings_completed()

    def _restore_settings(self, plugin_settings, instance_settings):
        qDebug('PluginHandlerXEmbedContainer._restore_settings()')
        self._dbus_plugin_settings_service.set_settings(plugin_settings)
        self._dbus_instance_settings_service.set_settings(instance_settings)
        self._dbus_container_service.restore_settings()

    def emit_restore_settings_completed(self):
        self._dbus_plugin_settings_service.set_settings(None)
        self._dbus_instance_settings_service.set_settings(None)
        super(PluginHandlerXEmbedContainer, self).emit_restore_settings_completed()

    def _trigger_configuration(self):
        self._dbus_container_service.trigger_configuration()

    def embed_widget(self, pid, widget_object_name):
        dock_widget = self._create_dock_widget()
        embed_container = QX11EmbedContainer(dock_widget)
        dock_widget.setWidget(embed_container)
        # every dock widget needs a unique name for save/restore geometry/state to work
        dock_widget.setObjectName(self._instance_id.tidy_str() + '__' + widget_object_name)
        #embed_container.clientClosed.connect(self._emit_close_signal)
        self._add_dock_widget(dock_widget, embed_container)
        # update widget title is triggered by client after embedding
        self._embed_containers[widget_object_name] = embed_container
        return embed_container.winId()

    def update_embedded_widget_title(self, widget_object_name, title):
        embed_container = self._embed_containers[widget_object_name]
        embed_container.setWindowTitle(title)

    def unembed_widget(self, widget_object_name):
        embed_container = self._embed_containers[widget_object_name]
        self.remove_widget(embed_container)
        del self._embed_containers[widget_object_name]
