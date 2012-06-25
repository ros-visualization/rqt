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

from . import qt_binding_helper  # @UnusedImport
from QtCore import QObject, QSignalMapper, Signal
from QtGui import QAction, QIcon, QMenu

from .menu_manager import MenuManager


class PluginMenu(QObject):

    """Menu of available plugins to load and running plugin instances to unload."""

    load_plugin_signal = Signal(str)
    unload_plugin_signal = Signal(str)

    def __init__(self, menu_bar, plugin_manager):
        super(PluginMenu, self).__init__()
        self.setObjectName('PluginMenu')

        plugin_menu = menu_bar.addMenu(menu_bar.tr('Plugins'))
        running_menu = menu_bar.addMenu(menu_bar.tr('Running'))
        self._plugin_menu_manager = MenuManager(plugin_menu)
        self._plugin_mapper = QSignalMapper(plugin_menu)
        self._plugin_mapper.mapped[str].connect(self.load_plugin_signal)
        self._running_menu_manager = MenuManager(running_menu)
        self._running_mapper = QSignalMapper(running_menu)
        self._running_mapper.mapped[str].connect(self.unload_plugin_signal)

        self._instances = {}

    def add_plugin(self, plugin_descriptor):
        base_path = plugin_descriptor.attributes().get('plugin_path')

        menu_manager = self._plugin_menu_manager
        # create submenus
        for group in plugin_descriptor.groups():
            label = group['label']
            if menu_manager.contains_menu(label):
                submenu = menu_manager.get_menu(label)
            else:
                submenu = QMenu(label, menu_manager.menu)
                menu_action = submenu.menuAction()
                self._enrich_action(menu_action, group, base_path)
                menu_manager.add_item(submenu)
            menu_manager = MenuManager(submenu)
        # create action
        action_attributes = plugin_descriptor.action_attributes()
        action = QAction(action_attributes['label'], menu_manager.menu)
        self._enrich_action(action, action_attributes, base_path)

        self._plugin_mapper.setMapping(action, plugin_descriptor.plugin_id())
        action.triggered.connect(self._plugin_mapper.map)

        not_available = plugin_descriptor.attributes().get('not_available')
        if not_available:
            action.setEnabled(False)
            action.setStatusTip(self.tr('Plugin is not available: %s') % not_available)

        # add action to menu
        menu_manager.add_item(action)

    def add_plugin_prefix(self, plugin_descriptor):
        action_attributes = plugin_descriptor.action_attributes()
        action = QAction(action_attributes['label'], self._plugin_menu_manager.menu)
        self._enrich_action(action, action_attributes)
        self._plugin_mapper.setMapping(action, plugin_descriptor.plugin_id())
        action.triggered.connect(self._plugin_mapper.map)
        self._plugin_menu_manager.add_prefix(action)

    def add_instance(self, plugin_descriptor, instance_id):
        action_attributes = plugin_descriptor.action_attributes()
        # create action
        label = self.tr('Close:') + ' ' + action_attributes['label']
        if instance_id.serial_number != 1:
            label = label + ' (%s)' % str(instance_id.serial_number)
        action = QAction(label, self._running_menu_manager.menu)
        base_path = plugin_descriptor.attributes().get('plugin_path')
        self._enrich_action(action, action_attributes, base_path)

        self._running_mapper.setMapping(action, str(instance_id))
        action.triggered.connect(self._running_mapper.map)

        self._running_menu_manager.add_item(action)
        self._instances[instance_id] = action

    def remove_instance(self, instance_id):
        action = self._instances[instance_id]
        self._running_mapper.removeMappings(action)
        self._running_menu_manager.remove_item(action)

    def _enrich_action(self, action, action_attributes, base_path=None):
        icontype = action_attributes.get('icontype', 'file')
        if 'icon' in action_attributes and action_attributes['icon'] is not None:
            if icontype == 'file':
                path = action_attributes['icon']
                if base_path is not None:
                    path = os.path.join(base_path, path)
                icon = QIcon(path)
                if len(icon.availableSizes()) == 0:
                    raise UserWarning('icon "%s" not found' % str(path))
            elif icontype == 'resource':
                icon = QIcon(action_attributes['icon'])
                if len(icon.availableSizes()) == 0:
                    raise UserWarning('icon "%s" not found' % str(path))
            elif icontype == 'theme':
                # see http://standards.freedesktop.org/icon-naming-spec/icon-naming-spec-latest.html
                icon = QIcon.fromTheme(action_attributes['icon'])
            else:
                raise UserWarning('unknown icon type "%s"' % str(icontype))
            action.setIcon(icon)

        if 'statustip' in action_attributes:
            action.setStatusTip(action_attributes['statustip'])
