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

import QtBindingHelper #@UnusedImport
from QtCore import QObject, Qt

from DockWidget import DockWidget
from PluginDescriptor import PluginDescriptor
from ReparentEvent import ReparentEvent

class ContainerManager(QObject):

    """Manager of `DockWidgetContainer`s enabling reparenting to stored parent."""

    def __init__(self, parent=None):
        super(ContainerManager, self).__init__(parent)
        self._container_descriptor = PluginDescriptor('__DockWidgetContainer')
        self._container_descriptor.set_action_attributes(self.tr('Container'), self.tr('Container for other dock widgets'))
        self._containers = {}


    def get_container_descriptor(self):
        return self._container_descriptor

    def add_to_plugin_menu(self, plugin_menu):
        plugin_menu.add_plugin_prefix(self._container_descriptor)


    def add_container(self, container):
        self._containers[container.serial_number()] = container

    def remove_container(self, container):
        del self._containers[container.serial_number()]

    def get_container(self, serial_number):
        if serial_number in self._containers:
            return self._containers[serial_number]
        return None


    def reparent_dock_widget_children(self, old_parent, new_parent):
        for child in old_parent.children():
            if isinstance(child, DockWidget):
                area = old_parent.dockWidgetArea(child)
                new_parent.addDockWidget(area, child)


    def event(self, e):
        if e.type() == ReparentEvent.reparent_event_type:
            #print 'ContainerManager.event()', 'reparent event', 'new parent:', e.new_parent.objectName()
            e.new_parent.addDockWidget(Qt.BottomDockWidgetArea, e.dock_widget)
            e.accept()
            return True
        return super(ContainerManager, self).event(e)
