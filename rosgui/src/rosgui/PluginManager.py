import os, traceback

import QtBindingHelper #@UnusedImport
from QtCore import qCritical, qDebug, QEvent, QObject, QSignalMapper, Qt, Signal, Slot
from QtGui import QAction, QIcon, QMenu

from MainWindowInterface import MainWindowInterface
from MenuManager import MenuManager
from PluginContext import PluginContext

class PluginManager(QObject):

    plugins_about_to_change_signal = Signal()
    plugins_changed_signal = Signal()
    plugin_help_signal = Signal(object)
    _deferred_reload_plugin_signal = Signal(str, int)

    def __init__(self, main_window, plugin_menu, running_menu, plugin_provider, hide_close_button=False):
        super(PluginManager, self).__init__()
        self.setObjectName('PluginManager')

        self._global_settings = None
        self._perspective_settings = None
        self._plugin_descriptors = {}
        self._running_plugins = {}
        self._hide_close_button_flag = hide_close_button

        self._main_window = main_window
        self._plugin_menu_manager = MenuManager(plugin_menu)
        self._running_menu_manager = MenuManager(running_menu)
        self._plugin_provider = plugin_provider

        self._plugin_mapper = QSignalMapper(plugin_menu)
        self._plugin_mapper.mapped[str].connect(self.load_plugin)

        self._running_mapper = QSignalMapper(running_menu)
        self._running_mapper.mapped[str].connect(self.unload_plugin)

        plugin_descriptors = self._plugin_provider.discover()

        # force connection type to queued, to delay the 'reloading' giving the 'unloading' time to finish
        self._deferred_reload_plugin_signal.connect(self._load_and_restore_plugin, type=Qt.QueuedConnection)

        # register plugins
        for plugin_descriptor in plugin_descriptors:
            self._plugin_descriptors[plugin_descriptor.plugin_id()] = plugin_descriptor

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

            not_available = plugin_descriptor.attributes().get('not_available', '')
            if not_available:
                action.setEnabled(False)
                action.setStatusTip(self.tr('Plugin is not available (%s) - may be it must be build?') % not_available)

            # add action to menu
            menu_manager.add_item(action)


    def find_plugins_by_name(self, lookup_name):
        plugins = {}
        for plugin_id, plugin_full_name in self.get_plugins().items():
            if plugin_full_name.lower().find(lookup_name.lower()) >= 0:
                plugins[plugin_id] = plugin_full_name
        return plugins


    def get_plugins(self):
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

        main_window_interface = MainWindowInterface(self._main_window, instance_id, self._hide_close_button_flag)
        main_window_interface.reload_plugin_instance_signal.connect(self.reload_plugin)
        main_window_interface.plugin_help_signal.connect(self._relay_plugin_help_signal)

        plugin_context = PluginContext()
        plugin_context.set_main_window(main_window_interface)
        plugin_context.set_serial_number(serial_number)

        try:
            instance = self._plugin_provider.load(plugin_id, plugin_context)
            if instance is None:
                raise Exception('PluginProvider "%s" returned None', type(self._plugin_provider))

        except Exception:
            qCritical('PluginManager._load_plugin(%s) failed:\n%s' % (plugin_id, traceback.format_exc()))

        else:
            qDebug('PluginManager._load_plugin(%s, %d) successful' % (plugin_id, serial_number))
            # set plugin instance for custom titlebar callbacks
            main_window_interface.set_plugin_instance(instance)
            self._add_running_plugin(plugin_id, serial_number, instance, main_window_interface)
            # restore settings after load
            self._call_method_on_plugin(instance_id, 'restore_settings')


    def _add_running_plugin(self, plugin_id, serial_number, instance, main_window_interface):
        plugin_descriptor = self._plugin_descriptors[plugin_id]
        action_attributes = plugin_descriptor.action_attributes()
        # create action
        label = self.tr('Close:') + ' ' + action_attributes['label']
        if serial_number != 1:
            label = label + ' (%s)' % str(serial_number)
        action = QAction(label, self._running_menu_manager.menu)
        base_path = plugin_descriptor.attributes().get('plugin_path')
        self._enrich_action(action, action_attributes, base_path)

        instance_id = self._build_instance_id(plugin_id, serial_number)
        self._running_mapper.setMapping(action, instance_id)
        action.triggered.connect(self._running_mapper.map)

        self._running_menu_manager.add_item(action)

        # store instance id to identify plugin
        instance.setProperty('rosgui.PluginManager.instance_id', instance_id)

        # trigger unload when deferred delete event for plugin is received
        instance.installEventFilter(self)

        info = {
            'plugin_id': plugin_id,
            'serial_number': serial_number,
            'instance': instance,
            'main_window_interface': main_window_interface,
            'action': action,
        }
        self._running_plugins[instance_id] = info


    def eventFilter(self, watched, event):
        if event.type() == QEvent.DeferredDelete:
            instance_id = watched.property('rosgui.PluginManager.instance_id')
            if instance_id in self._running_plugins:
                self.unload_plugin(instance_id)
                # TOOD: check if ignore() is necessary
                return True
        return QObject.eventFilter(self, watched, event)


    def _unload_plugin(self, instance_id):
        # convert from unicode
        instance_id = str(instance_id)

        # save settings before unloading
        self._call_method_on_plugin(instance_id, 'save_settings')

        # garbage references
        info = self._running_plugins[instance_id]
        self._running_mapper.removeMappings(info['action'])
        self._running_menu_manager.remove_item(info['action'])
        self._running_plugins.pop(instance_id)

        # destroy instance and unload plugin
        instance = info['instance']
        instance.removeEventFilter(self)
        try:
            if not hasattr(instance, 'close_plugin'):
                raise NotImplementedError('method "close_plugin" not implemented by plugin')
            instance.close_plugin()
        except NotImplementedError:
            qCritical('PluginManager._unload_plugin() plugin "%s" must implement close_plugin method' % str(info['plugin_id']))
        instance.deleteLater()
        self._plugin_provider.unload(instance)
        qDebug('PluginManager._unload_plugin() successful')


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
            action.setIconVisibleInMenu(True)


    def save_settings(self, global_settings, perspective_settings):
        qDebug('PluginManager.save_settings()')
        self._global_settings = global_settings.get_settings('pluginmanager')
        self._perspective_settings = perspective_settings.get_settings('pluginmanager')
        self._store_running_plugins()
        # delegate call to all running plugins
        self._call_method_on_all_plugins('save_settings')


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
                self._call_method_on_plugin(instance_id, 'restore_settings')
        # load not yet loaded plugins
        for instance_id, info in plugins.items():
            if instance_id not in self._running_plugins:
                self._load_plugin(info['plugin_id'], info['serial_number'])


    def _call_method_on_all_plugins(self, method_name):
        for instance_id in self._running_plugins.keys():
            self._call_method_on_plugin(instance_id, method_name)

    def _call_method_on_plugin(self, instance_id, method_name):
        if instance_id in self._running_plugins:
            info = self._running_plugins[instance_id]
            global_settings_instance = self._global_settings.get_settings('plugin ' + instance_id)
            perspective_settings_instance = self._perspective_settings.get_settings('plugin ' + instance_id)

            instance = info['instance']
            if hasattr(instance, method_name):
                method = getattr(instance, method_name)
                try:
                    global_settings_plugin = global_settings_instance.get_settings('plugin')
                    perspective_settings_plugin = perspective_settings_instance.get_settings('plugin')
                    method(global_settings_plugin, perspective_settings_plugin)
                except Exception:
                    qCritical('PluginManager._call_method_on_plugin(%s, %s) failed:\n%s' % (str(info['plugin_id']), method_name, traceback.format_exc()))

            # call after instance method since the plugin may spawn additional dock widgets depending on current settings
            main_window_interface = info['main_window_interface']
            method = getattr(main_window_interface, method_name)
            method(perspective_settings_instance)
