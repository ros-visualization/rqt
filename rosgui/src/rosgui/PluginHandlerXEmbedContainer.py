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

import os, sys

from PluginHandler import PluginHandler
from PluginHandlerDBusInterface import PluginHandlerDBusInterface
from QtCore import qDebug, QProcess, qWarning
from QtGui import QX11EmbedContainer
from rosgui_main import rosgui_main_filename

class PluginHandlerXEmbedContainer(PluginHandler):

    def __init__(self, main_window, instance_id, plugin_id, serial_number, application_context, dbus_object_path):
        super(PluginHandlerXEmbedContainer, self).__init__(main_window, instance_id, plugin_id, serial_number, application_context)
        self.setObjectName('PluginHandlerXEmbedContainer')

        self._dbus_object_path = dbus_object_path
        self._dbus_server = None
        self._process = None
        self._pid = None

    def _load(self):
        self._dbus_server = PluginHandlerDBusInterface(self, self._application_context, self._dbus_object_path)
        self._process = QProcess(self)
        self._process.setProcessChannelMode(QProcess.SeparateChannels)
        self._process.readyReadStandardOutput.connect(self._print_process_output)
        self._process.readyReadStandardError.connect(self._print_process_error)
        self._process.finished.connect(self.close_plugin)
        # start python with unbuffered stdout/stderr so that the order of the output is retained
        cmd = sys.executable + ' -u'
        cmd += ' %s' % rosgui_main_filename()
        if self._application_context.options.qt_binding is not None:
            cmd += ' --qt-binding=%s' % self._application_context.options.qt_binding
        cmd += ' --embed-plugin=%s --embed-plugin-pid=%d --embed-plugin-serial=%s' % (self._plugin_id, os.getpid() , self._serial_number)
        #qDebug('PluginHandlerXEmbedContainer._load() starting command: %s' % cmd)
        self._process.start(cmd)
        started = self._process.waitForStarted(5000)
        if not started:
            self._dbus_server.remove_from_connection()
            return None
        # QProcess.pid() has been added to PySide in 1.0.5
        if hasattr(self._process, 'pid'):
            self._pid = self._process.pid()
        qDebug('PluginHandlerXEmbedContainer._load() started subprocess%s for plugin "%s"' % (' (#%d)' % self._pid if self._pid is not None else '', self._instance_id))
        return True

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

    def add_embed_dock_widget(self, pid, area):
        dock_widget = self._create_dock_widget()
        embed_container = QX11EmbedContainer(dock_widget)
        embed_container.clientClosed.connect(self._emit_close_signal)
        dock_widget.setWidget(embed_container)
        self._add_dock_widget(dock_widget, area)
        self.update_widget_title(embed_container)
        return embed_container.winId()

    def _shutdown_plugin(self):
        self._process.finished.disconnect(self.close_plugin)
        self._process.close()

    def unload(self):
        self._process.waitForFinished(5000)
        if self._process.state() != QProcess.NotRunning:
            self._process.kill()
        self._process = None
        self._dbus_server.remove_from_connection()
