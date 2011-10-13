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
    _deferred_load_plugin_signal = Signal(str, int)

    def __init__(self, main_window, plugin_menu, running_menu, plugin_provider, hide_close_button=False):
        super(PluginManager, self).__init__()
        self.setObjectName('PluginManager')

        self.global_settings_ = None
        self.perspective_settings_ = None
        self.plugin_descriptors_ = {}
        self.running_plugins_ = {}
        self.hide_close_button = hide_close_button

        self.main_window_ = main_window
        self.plugin_menu_manager_ = MenuManager(plugin_menu)
        self.running_menu_manager_ = MenuManager(running_menu)
        self.plugin_provider_ = plugin_provider

        self.plugin_mapper_ = QSignalMapper(plugin_menu)
        self.plugin_mapper_.mapped[str].connect(self.load_plugin)

        self.running_mapper_ = QSignalMapper(running_menu)
        self.running_mapper_.mapped[str].connect(self.unload_plugin)

        plugin_descriptors = self.plugin_provider_.discover()

        # force connection type to queued, to delay the 'reloading' giving the 'unloading' time to finish
        self._deferred_load_plugin_signal.connect(self.load_plugin, type=Qt.QueuedConnection)

        self.__register_plugins(plugin_descriptors)

    @Slot(str)
    def reload_plugin(self, instance_id):
        # unload plugin now
        self.unload_plugin(instance_id)

        plugin_id, serial_number = self.__split_instance_id(instance_id)
        serial_number = int(serial_number)
        # deferred call to load_plugin 
        self._deferred_load_plugin_signal.emit(plugin_id, serial_number)

    @Slot(str)
    def relay_plugin_help_signal(self, instance_id):
        plugin_id, _ = self.__split_instance_id(instance_id)
        plugin_descriptor = self.plugin_descriptors_[plugin_id]
        self.plugin_help_signal.emit(plugin_descriptor)

    def find_plugin_by_name(self, lookup_name):
        found_plugins = {}
        for plugin_id, plugin_descriptor in self.plugin_descriptors_.items():

            plugin_name_parts = []
            plugin_name = plugin_descriptor.attributes().get('plugin_name', None)
            if plugin_name is not None:
                plugin_name_parts.append(plugin_name)
            plugin_name_parts += plugin_descriptor.attributes().get('class_type', 'unknown').split('::')

            plugin_full_name = '/'.join(plugin_name_parts)

            if plugin_full_name.lower().find(lookup_name) >= 0:
                found_plugins[plugin_id] = plugin_full_name

        return found_plugins

    @Slot(str)
    @Slot(str, int)
    def load_plugin(self, plugin_id, serial_number=None):
        # convert from unicode
        plugin_id = str(plugin_id)

        if serial_number is None:
            serial_number = self.__next_serial_number(plugin_id)
        instance_id = self.__build_instance_id(plugin_id, serial_number)

        # if the requested instance is already running, so nothing
        if instance_id in self.running_plugins_:
            return

        try:
            main_window_interface = MainWindowInterface(self.main_window_, instance_id, self.hide_close_button)
            main_window_interface.reload_plugin_instance_signal.connect(self.reload_plugin)
            main_window_interface.plugin_help_signal.connect(self.relay_plugin_help_signal)

            plugin_context = PluginContext()
            plugin_context.set_main_window(main_window_interface)
            plugin_context.set_serial_number(serial_number)

            instance = self.plugin_provider_.load(plugin_id, plugin_context)
            if instance is None:
                raise Exception('load returned None')

        except Exception:
            qCritical('PluginManager.load_plugin(%s) failed:\n%s' % (plugin_id, traceback.format_exc()))

        else:
            qDebug('PluginManager.load_plugin(%s) successful' % plugin_id)
            # set plugin instance for custom titlebar callbacks
            main_window_interface.set_plugin_instance(instance)
            self.__add_running_plugin(plugin_id, serial_number, instance, main_window_interface)
            # restore settings after load
            self.__call_method_on_plugin(instance_id, 'restore_settings')
            self.plugins_changed_signal.emit()

    @Slot(str)
    def unload_plugin(self, instance_id):
        # convert from unicode
        instance_id = str(instance_id)
        # trigger save of top-level widget setup
        self.plugins_about_to_change_signal.emit()
        # save settings before unloading
        self.__call_method_on_plugin(instance_id, 'save_settings')
        self.__unload_plugin(instance_id)

    def __unload_plugin(self, instance_id):
        # destroy instance and unload plugin
        info = self.running_plugins_[instance_id]
        self.__remove_running_plugin(info['action'])
        self.running_plugins_.pop(instance_id)

        instance = info['instance']
        instance.removeEventFilter(self)
        try:
            if not hasattr(instance, 'close_plugin'):
                raise NotImplementedError('method not implemented')
            instance.close_plugin()
        except NotImplementedError:
            qCritical('PluginManager.__unload_plugin() plugin "%s" must implement close_plugin method' % str(info['plugin_id']))
        instance.deleteLater()
        self.plugin_provider_.unload(instance)
        qDebug('PluginManager.__unload_plugin() successful')

    def __register_plugins(self, plugin_descriptors):
        for plugin_descriptor in plugin_descriptors:
            self.plugin_descriptors_[plugin_descriptor.plugin_id()] = plugin_descriptor

            base_path = plugin_descriptor.attributes().get('plugin_path')

            menu_manager = self.plugin_menu_manager_
            # create submenus
            for group in plugin_descriptor.groups():
                label = group['label']
                if menu_manager.contains_menu(label):
                    submenu = menu_manager.get_menu(label)
                else:
                    submenu = QMenu(label, menu_manager.menu())
                    menu_action = submenu.menuAction()
                    self.__enrich_action(menu_action, group, base_path)
                    menu_manager.add_item(submenu)
                menu_manager = MenuManager(submenu)
            # create action
            action_attributes = plugin_descriptor.action_attributes()
            action = QAction(action_attributes['label'], menu_manager.menu())
            self.__enrich_action(action, action_attributes, base_path)

            self.plugin_mapper_.setMapping(action, plugin_descriptor.plugin_id())
            action.triggered.connect(self.plugin_mapper_.map)

            not_available = plugin_descriptor.attributes().get('not_available', '')
            if not_available:
                action.setEnabled(False)
                action.setStatusTip(self.tr('Plugin is not available (%s) - may be it must be build?') % not_available)

            # add action to menu
            menu_manager.add_item(action)

    def __add_running_plugin(self, plugin_id, serial_number, instance, main_window_interface):
        plugin_descriptor = self.plugin_descriptors_[plugin_id]
        action_attributes = plugin_descriptor.action_attributes()
        # create action
        label = 'Close: ' + action_attributes['label']
        if serial_number != 1:
            label = label + ' (%s)' % str(serial_number)
        action = QAction(label, self.running_menu_manager_.menu())
        base_path = plugin_descriptor.attributes().get('plugin_path')
        self.__enrich_action(action, action_attributes, base_path)

        instance_id = self.__build_instance_id(plugin_id, serial_number)
        self.running_mapper_.setMapping(action, instance_id)
        action.triggered.connect(self.running_mapper_.map)

        self.running_menu_manager_.add_item(action)

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
        self.running_plugins_[instance_id] = info

    def eventFilter(self, watched, event):
        if event.type() == QEvent.DeferredDelete:
            instance_id = watched.property('rosgui.PluginManager.instance_id')
            if self.running_plugins_.has_key(instance_id):
                self.unload_plugin(instance_id)
                # TOOD: check if ignore() is necessary
                return True
        return QObject.eventFilter(self, watched, event)

    def __remove_running_plugin(self, action):
        self.running_mapper_.removeMappings(action)
        self.running_menu_manager_.remove_item(action)

    def __next_serial_number(self, plugin_id):
        # collect serial numbers of all running instances of the specific plugin
        used_serial_numbers = {}
        for info in self.running_plugins_.values():
            if info['plugin_id'] == plugin_id:
                used_serial_numbers[info['serial_number']] = None

        # find first non-used serial number
        serial_number = 1
        while used_serial_numbers.has_key(serial_number):
            serial_number = serial_number + 1
        return serial_number

    def __build_instance_id(self, plugin_id, serial_number):
        return plugin_id + '#' + str(serial_number)

    def __split_instance_id(self, instance_id):
        return instance_id.rsplit('#', 1)

    def __enrich_action(self, action, action_attributes, base_path=None):
        self.__set_icon(action, action_attributes, base_path)
        if action_attributes.has_key('statustip'):
            action.setStatusTip(action_attributes['statustip'])

    def __set_icon(self, action, action_attributes, base_path=None):
        icontype = action_attributes.get('icontype', 'file')
        if action_attributes.has_key('icon') and action_attributes['icon'] is not None:
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
        self.global_settings_ = global_settings.get_settings('pluginmanager')
        self.perspective_settings_ = perspective_settings.get_settings('pluginmanager')
        self.__store_running_plugins()
        self.__save_plugin_settings()

    def restore_settings(self, global_settings, perspective_settings):
        qDebug('PluginManager.restore_settings()')
        self.global_settings_ = global_settings.get_settings('pluginmanager')
        self.perspective_settings_ = perspective_settings.get_settings('pluginmanager')
        self.__restore_running_plugins()
        self.plugins_changed_signal.emit()

    def __store_running_plugins(self):
        if self.perspective_settings_ is not None:
            plugins = {}
            for info in self.running_plugins_.values():
                plugin_id = info['plugin_id']
                if not plugins.has_key(plugin_id):
                    plugins[plugin_id] = []
                plugins[plugin_id].append(info['serial_number'])
            self.perspective_settings_.set_value('running-plugins', plugins)

    def __restore_running_plugins(self):
        plugins = {}
        if self.perspective_settings_ is not None:
            data = self.perspective_settings_.value('running-plugins', {})
            for plugin_id, serial_numbers in data.items():
                for serial_number in serial_numbers:
                    info = {
                        'plugin_id': plugin_id,
                        'serial_number': serial_number,
                    }
                    instance_id = self.__build_instance_id(plugin_id, serial_number)
                    plugins[instance_id] = info

        # unload obsolete plugins
        for instance_id in self.running_plugins_.keys():
            if not plugins.has_key(instance_id):
                self.__unload_plugin(instance_id)

        # restore settings for already loaded plugins
        for instance_id, info in plugins.items():
            if self.running_plugins_.has_key(instance_id):
                self.__call_method_on_plugin(instance_id, 'restore_settings')

        # load not yet loaded plugins
        for instance_id, info in plugins.items():
            if not self.running_plugins_.has_key(instance_id):
                self.load_plugin(info['plugin_id'], info['serial_number'])

    def __save_plugin_settings(self):
        # delegate call to all running plugins
        self.__call_method_on_all_plugins('save_settings')

    def __call_method_on_all_plugins(self, method_name):
        for instance_id in self.running_plugins_.keys():
            self.__call_method_on_plugin(instance_id, method_name)

    def __call_method_on_plugin(self, instance_id, method_name):
        if self.running_plugins_.has_key(instance_id):
            info = self.running_plugins_[instance_id]
            global_settings_instance = self.global_settings_.get_settings(instance_id)
            perspective_settings_instance = self.perspective_settings_.get_settings(instance_id)

            instance = info['instance']
            if hasattr(instance, method_name):
                method = getattr(instance, method_name)
                try:
                    global_settings_plugin = global_settings_instance.get_settings('plugin')
                    perspective_settings_plugin = perspective_settings_instance.get_settings('plugin')
                    method(global_settings_plugin, perspective_settings_plugin)
                except Exception:
                    qCritical('PluginManager.__call_method_on_plugin(%s, %s) failed:\n%s' % (str(info['plugin_id']), method_name, traceback.format_exc()))

            # call after instance method since the plugin may spawn additional dock widgets depending on current settings 
            main_window_interface = info['main_window_interface']
            if hasattr(main_window_interface, method_name):
                method = getattr(main_window_interface, method_name)
                try:
                    method(perspective_settings_instance)
                except Exception:
                    qCritical('PluginManager.__call_method_on_plugin(%s, %s) call on main_window_interface failed:\n%s' % (str(info['plugin_id']), method_name, traceback.format_exc()))
