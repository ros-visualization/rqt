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

from dbus import Interface, SessionBus
from PluginHandler import PluginHandler
from QtGui import QVBoxLayout, QX11EmbedWidget

class PluginHandlerXEmbedClient(PluginHandler):

    def __init__(self, main_window, instance_id, plugin_id, serial_number, application_context, dbus_object_path):
        super(PluginHandlerXEmbedClient, self).__init__(main_window, instance_id, plugin_id, serial_number, application_context)
        self.setObjectName('PluginHandlerXEmbedClient')

        self._dbus_object_path = dbus_object_path
        self._remote_object = None
        self._remote_interface = None
        self._plugin = None
        self._embedded_widgets = {}

    def _load(self):
        self._remote_object = SessionBus().get_object(self._application_context.dbus_host_bus_name, self._dbus_object_path)
        self._remote_interface = Interface(self._remote_object, 'org.ros.rosgui.PluginHandlerXEmbed')
        self._plugin = super(PluginHandlerXEmbedClient, self)._load()
        return self._plugin

    def add_widget(self, widget, area):
        embed_widget = QX11EmbedWidget()
        layout = QVBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(widget)
        embed_widget.setLayout(layout)

        # close embed widget when container is closed
        embed_widget.containerClosed.connect(embed_widget.close)

        embed_container_window_id = self._remote_interface.embed_widget(os.getpid(), widget.objectName(), area)
        embed_widget.embedInto(embed_container_window_id)

        self._embedded_widgets[widget] = embed_widget
        self.update_widget_title(widget)

        embed_widget.show()

    def _update_widget_title(self, widget, title):
        self._remote_interface.update_embedded_widget_title(widget.objectName(), title)

    def remove_widget(self, widget):
        self._remote_interface.unembed_widget(widget.objectName())
        # do not delete the widget, only the embed widget
        widget.setParent(None)
        del self._embedded_widgets[widget]

    def close_plugin(self):
        self._remote_interface.close_plugin()
