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
from QtCore import qCritical, qDebug, QObject, QSignalMapper, Qt, Signal, Slot
from QtGui import QAction, QIcon, QMenu

from MenuManager import MenuManager
from PluginHandler import PluginHandler
from PluginHandlerXEmbed import PluginHandlerXEmbed
from PluginManagerDBusInterface import PluginManagerDBusInterface

class PluginManager(QObject):

    plugins_about_to_change_signal = Signal()
    plugins_changed_signal = Signal()
    plugin_help_signal = Signal(object)
    _deferred_reload_plugin_signal = Signal(str, int)

    def __init__(self, plugin_provider, application_context):
        super(PluginManager, self).__init__()
        self.setObjectName('PluginManager')

        self._plugin_provider = plugin_provider
        self._application_context = application_context

        self._main_window = None
        self._plugin_menu_manager = None
        self._plugin_mapper = None
        self._running_menu_manager = None
        self._running_mapper = None

        self._global_settings = None
        self._perspective_settings = None
        self._plugin_descriptors = None
        self._running_plugins = {}

        # force connection type to queued, to delay the 'reloading' giving the 'unloading' time to finish
        self._deferred_reload_plugin_signal.connect(self._load_and_restore_plugin, type=Qt.QueuedConnection)

        if application_context.dbus_unique_bus_name is not None:
            self._dbus_server = PluginManagerDBusInterface(self, self._application_context)


    def set_main_window(self, main_window):
        self._main_window = main_window
        menu_bar = self._main_window.menuBar()
        plugin_menu = menu_bar.addMenu(menu_bar.tr('Plugins'))
        running_menu = menu_bar.addMenu(menu_bar.tr('Running'))
        self._plugin_menu_manager = MenuManager(plugin_menu)
        self._plugin_mapper = QSignalMapper(plugin_menu)
        self._plugin_mapper.mapped[str].connect(self.load_plugin)
        self._running_menu_manager = MenuManager(running_menu)
        self._running_mapper = QSignalMapper(running_menu)
        self._running_mapper.mapped[str].connect(self.unload_plugin)


    def discover(self):
        # skip discover if called multiple times
        if self._plugin_descriptors is not None:
            return
        self._plugin_descriptors = {}
        # register discovered plugins
        plugin_descriptors = self._plugin_provider.discover()
        for plugin_descriptor in plugin_descriptors:
            self._plugin_descriptors[plugin_descriptor.plugin_id()] = plugin_descriptor

            if self._plugin_menu_manager is None:
                continue

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


    def find_plugins_by_name(self, lookup_name):
        plugins = {}
        for plugin_id, plugin_full_name in self.get_plugins().items():
            if plugin_full_name.lower().find(lookup_name.lower()) >= 0:
                plugins[plugin_id] = plugin_full_name
        return plugins


    def get_plugins(self):
        if self._plugin_descriptors is None:
            self.discover()
        plugins = {}
        for plugin_id, plugin_descriptor in self._plugin_descriptors.items():
            plugin_name_parts = []
            plugin_name = plugin_descriptor.attributes().get('plugin_name', None)
            if plugin_name is not None:
                plugin_name_parts.append(plugin_name)
            plugin_name_parts += plugin_descriptor.attributes().get('class_type', 'unknown').split('::')
            plugin_full_name = '/'.join(plugin_name_parts)
            plugins[plugin_id] = plugin_full_name
        return plugins


    def is_plugin_running(self, plugin_id, serial_number):
        instance_id = self._build_instance_id(plugin_id, serial_number)
        return instance_id in self._running_plugins


    @Slot(str)
    @Slot(str, int)
    def load_plugin(self, plugin_id, serial_number=None):
        # save state of top-level widgets
        self.plugins_about_to_change_signal.emit()
        if serial_number is None:
            serial_number = self._next_serial_number(plugin_id)
        self._load_and_restore_plugin(plugin_id, serial_number)


    def _next_serial_number(self, plugin_id):
        # convert from unicode
        plugin_id = str(plugin_id)
        # collect serial numbers of all running instances of the specific plugin
        used_serial_numbers = {}
        for info in self._running_plugins.values():
            if info['plugin_id'] == plugin_id:
                used_serial_numbers[info['serial_number']] = None

        # find first non-used serial number
        serial_number = 1
        while serial_number in used_serial_numbers:
            serial_number = serial_number + 1
        return serial_number


    @Slot(str)
    def unload_plugin(self, instance_id):
        # save state of top-level widgets
        self.plugins_about_to_change_signal.emit()
        self._unload_plugin(instance_id)


    @Slot(str)
    def reload_plugin(self, instance_id):
        # save state of top-level widgets
        self.plugins_about_to_change_signal.emit()
        self._unload_plugin(instance_id)
        # deferred call to reload_plugin
        plugin_id, serial_number = self._split_instance_id(instance_id)
        self._deferred_reload_plugin_signal.emit(plugin_id, serial_number)


    @Slot(str, int)
    def _load_and_restore_plugin(self, plugin_id, serial_number):
        # convert from unicode
        plugin_id = str(plugin_id)
        self._load_plugin(plugin_id, serial_number)
        # restore state of top-level widgets
        self.plugins_changed_signal.emit()


    def _load_plugin(self, plugin_id, serial_number):
        # convert from unicode
        plugin_id = str(plugin_id)

        # if the requested instance is already running, do nothing
        instance_id = self._build_instance_id(plugin_id, serial_number)
        if instance_id in self._running_plugins:
            raise Exception('PluginManager._load_plugin(%s, %d) instance already loaded' % (plugin_id, serial_number))

        if not self._application_context.options.multi_process and not self._application_context.options.embed_plugin:
            handler = PluginHandler(self._main_window, instance_id, plugin_id, serial_number, self._application_context)
        else:
            handler = PluginHandlerXEmbed(self._main_window, instance_id, plugin_id, serial_number, self._application_context)
        handler.close_signal.connect(self.unload_plugin)
        handler.reload_signal.connect(self.reload_plugin)
        handler.help_signal.connect(self._relay_plugin_help_signal)

        try:
            loaded = handler.load(self._plugin_provider)
            if not loaded:
                qCritical('PluginManager._load_plugin() could not load plugin "%s"' % plugin_id)
                return

        except Exception:
            qCritical('PluginManager._load_plugin(%s) failed:\n%s' % (plugin_id, traceback.format_exc()))
            # quit embed application in case of exceptions
            if self._application_context.options.embed_plugin:
                exit(-1)

        else:
            qDebug('PluginManager._load_plugin(%s, %d) successful' % (plugin_id, serial_number))
            # set plugin instance for custom titlebar callbacks
            self._add_running_plugin(plugin_id, serial_number, handler)
            # restore settings after load
            self._call_method_on_running_plugin(instance_id, 'restore_settings')


    def _add_running_plugin(self, plugin_id, serial_number, handler):
        info = {
            'plugin_id': plugin_id,
            'serial_number': serial_number,
            'handler': handler,
            'action': None,
        }

        instance_id = self._build_instance_id(plugin_id, serial_number)

        if self._running_menu_manager is not None:
            plugin_descriptor = self._plugin_descriptors[plugin_id]
            action_attributes = plugin_descriptor.action_attributes()
            # create action
            label = self.tr('Close:') + ' ' + action_attributes['label']
            if serial_number != 1:
                label = label + ' (%s)' % str(serial_number)
            action = QAction(label, self._running_menu_manager.menu)
            base_path = plugin_descriptor.attributes().get('plugin_path')
            self._enrich_action(action, action_attributes, base_path)

            self._running_mapper.setMapping(action, instance_id)
            action.triggered.connect(self._running_mapper.map)

            self._running_menu_manager.add_item(action)
            info['action'] = action

        self._running_plugins[instance_id] = info


    def _unload_plugin(self, instance_id):
        # convert from unicode
        instance_id = str(instance_id)

        # save settings before unloading
        self._call_method_on_running_plugin(instance_id, 'save_settings')

        # garbage references
        info = self._running_plugins[instance_id]
        if self._running_mapper is not None:
            self._running_mapper.removeMappings(info['action'])
        if self._running_menu_manager is not None:
            self._running_menu_manager.remove_item(info['action'])
        self._running_plugins.pop(instance_id)

        # shutdown and unload plugin
        handler = info['handler']
        handler.shutdown_plugin()
        handler.unload()
        qDebug('PluginManager._unload_plugin(%s) successful' % instance_id)


    def _build_instance_id(self, plugin_id, serial_number):
        return plugin_id + '#' + str(serial_number)


    def _split_instance_id(self, instance_id):
        # convert from unicode
        instance_id = str(instance_id)
        parts = instance_id.rsplit('#', 1)
        return [parts[0], int(parts[1])]


    @Slot(str)
    def _relay_plugin_help_signal(self, instance_id):
        plugin_id, _ = self._split_instance_id(instance_id)
        plugin_descriptor = self._plugin_descriptors[plugin_id]
        self.plugin_help_signal.emit(plugin_descriptor)


    def _enrich_action(self, action, action_attributes, base_path=None):
        self._set_icon(action, action_attributes, base_path)
        if 'statustip' in action_attributes:
            action.setStatusTip(action_attributes['statustip'])


    def _set_icon(self, action, action_attributes, base_path=None):
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


    def save_settings(self, global_settings, perspective_settings):
        qDebug('PluginManager.save_settings()')
        self._global_settings = global_settings.get_settings('pluginmanager')
        self._perspective_settings = perspective_settings.get_settings('pluginmanager')
        self._store_running_plugins()
        # delegate call to all running plugins
        for instance_id in self._running_plugins.keys():
            self._call_method_on_running_plugin(instance_id, 'save_settings')


    def restore_settings(self, global_settings, perspective_settings):
        qDebug('PluginManager.restore_settings()')
        self._global_settings = global_settings.get_settings('pluginmanager')
        self._perspective_settings = perspective_settings.get_settings('pluginmanager')
        self._restore_running_plugins()
        # restore state of top-level widgets
        self.plugins_changed_signal.emit()


    def _store_running_plugins(self):
        if self._perspective_settings is not None:
            plugins = {}
            for info in self._running_plugins.values():
                plugin_id = info['plugin_id']
                if plugin_id not in plugins:
                    plugins[plugin_id] = []
                plugins[plugin_id].append(info['serial_number'])
            self._perspective_settings.set_value('running-plugins', plugins)


    def _restore_running_plugins(self):
        plugins = {}
        if self._perspective_settings is not None:
            data = self._perspective_settings.value('running-plugins', {})
            for plugin_id, serial_numbers in data.items():
                for serial_number in serial_numbers:
                    info = {
                        'plugin_id': plugin_id,
                        'serial_number': serial_number,
                    }
                    instance_id = self._build_instance_id(plugin_id, serial_number)
                    plugins[instance_id] = info
        # unload obsolete plugins
        for instance_id in self._running_plugins.keys():
            if instance_id not in plugins:
                self._unload_plugin(instance_id)
        # restore settings for already loaded plugins
        for instance_id, info in plugins.items():
            if instance_id in self._running_plugins:
                self._call_method_on_running_plugin(instance_id, 'restore_settings')
        # load not yet loaded plugins
        for instance_id, info in plugins.items():
            if instance_id not in self._running_plugins:
                self._load_plugin(info['plugin_id'], info['serial_number'])


    def _call_method_on_running_plugin(self, instance_id, method_name):
        if instance_id in self._running_plugins and self._global_settings is not None and self._perspective_settings is not None:
            info = self._running_plugins[instance_id]
            global_settings = self._global_settings.get_settings('plugin ' + instance_id)
            perspective_settings = self._perspective_settings.get_settings('plugin ' + instance_id)
            handler = info['handler']
            method = getattr(handler, method_name)
            method(global_settings, perspective_settings)
