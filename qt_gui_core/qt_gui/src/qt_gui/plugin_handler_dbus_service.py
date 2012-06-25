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

from dbus.service import Object
import dbus


class PluginHandlerDBusService(Object):

    """DBus service for an indirect plugin handler, i.e. `PluginHandlerXEmbedContainer`."""

    def __init__(self, plugin_handler, object_path):
        super(PluginHandlerDBusService, self).__init__(object_path)
        self._plugin_handler = plugin_handler

    @dbus.service.method('org.ros.qt_gui.PluginHandlerContainer', in_signature='bb', out_signature='')
    def load_completed(self, loaded, has_configuration):
        self._plugin_handler.load_completed(loaded, has_configuration)

    @dbus.service.method('org.ros.qt_gui.PluginHandlerContainer', in_signature='is', out_signature='i')
    def embed_widget(self, pid, widget_object_name):
        return self._plugin_handler.embed_widget(pid, widget_object_name)

    @dbus.service.method('org.ros.qt_gui.PluginHandlerContainer', in_signature='ss', out_signature='')
    def update_embedded_widget_title(self, widget_object_name, title):
        self._plugin_handler.update_embedded_widget_title(widget_object_name, title)

    @dbus.service.method('org.ros.qt_gui.PluginHandlerContainer', in_signature='s', out_signature='')
    def unembed_widget(self, widget_object_name):
        self._plugin_handler.unembed_widget(widget_object_name)

    @dbus.service.method('org.ros.qt_gui.PluginHandlerContainer', in_signature='', out_signature='')
    def close_plugin(self):
        self._plugin_handler.close_plugin()

    @dbus.service.signal('org.ros.qt_gui.PluginHandlerContainer', signature='')
    def shutdown_plugin(self):
        pass

    @dbus.service.method('org.ros.qt_gui.PluginHandlerContainer', in_signature='', out_signature='')
    def shutdown_plugin_completed(self):
        self._plugin_handler.emit_shutdown_plugin_completed()

    @dbus.service.signal('org.ros.qt_gui.PluginHandlerContainer', signature='')
    def save_settings(self):
        pass

    @dbus.service.method('org.ros.qt_gui.PluginHandlerContainer', in_signature='', out_signature='')
    def save_settings_completed(self):
        self._plugin_handler.emit_save_settings_completed()

    @dbus.service.signal('org.ros.qt_gui.PluginHandlerContainer', signature='')
    def restore_settings(self):
        pass

    @dbus.service.method('org.ros.qt_gui.PluginHandlerContainer', in_signature='', out_signature='')
    def restore_settings_completed(self):
        self._plugin_handler.emit_restore_settings_completed()

    @dbus.service.signal('org.ros.qt_gui.PluginHandlerContainer', signature='')
    def trigger_configuration(self):
        pass
