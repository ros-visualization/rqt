import json, os, re
from pprint import pformat

from QtBindingHelper import loadUi
from QtCore import QByteArray, qDebug, QObject, QSignalMapper, Signal, Slot
from QtGui import QAction, QFileDialog, QIcon, QInputDialog, QMessageBox, QValidator

from MenuManager import MenuManager
from Settings import Settings
from SettingsProxy import SettingsProxy

class PerspectiveManager(QObject):

    perspective_changed_signal = Signal(basestring)
    save_settings_signal = Signal(Settings, Settings)
    restore_settings_signal = Signal(Settings, Settings)

    HIDDEN_PREFIX = '@'

    def __init__(self, settings, menu):
        super(PerspectiveManager, self).__init__()
        self.setObjectName('PerspectiveManager')

        self._settings_proxy = SettingsProxy(settings)
        self._global_settings = Settings(self._settings_proxy, 'global')
        self._perspective_settings = None
        self._create_perspective_dialog = None

        self._menu_manager = MenuManager(menu)
        self._perspective_mapper = QSignalMapper(menu)
        self._perspective_mapper.mapped[str].connect(self.switch_perspective)

        # get perspective list from settings
        self.perspectives = self._settings_proxy.value('', 'perspectives', [])
        if isinstance(self.perspectives, basestring):
            self.perspectives = [ self.perspectives ]

        self._current_perspective = None
        self._remove_action = None

        self._generate_menu()


    def set_perspective(self, name, hide_perspective=False):
        if name is None:
            name = self._settings_proxy.value('', 'current-perspective', 'Default')
        elif hide_perspective:
            name = self.HIDDEN_PREFIX + name
        self.switch_perspective(name)


    @Slot(str)
    @Slot(str, bool)
    @Slot(str, bool, bool)
    def switch_perspective(self, name, settings_changed=True, save_before=True):
        if save_before and self._global_settings is not None and self._perspective_settings is not None:
            self.save_settings_signal.emit(self._global_settings, self._perspective_settings)

        # convert from unicode
        name = str(name)

        qDebug('PerspectiveManager.switch_perspective() switching to perspective "%s"' % name)
        if self._current_perspective is not None:
            self._menu_manager.set_item_checked(self._current_perspective, False)
            self._menu_manager.set_item_disabled(self._current_perspective, False)

        # create perspective if necessary 
        if name not in self.perspectives:
            self._create_perspective(name, clone_perspective=False)

        # update current perspective
        self._current_perspective = name
        self._menu_manager.set_item_checked(self._current_perspective, True)
        self._menu_manager.set_item_disabled(self._current_perspective, True)
        if not self._current_perspective.startswith(self.HIDDEN_PREFIX):
            self._settings_proxy.set_value('', 'current-perspective', self._current_perspective)
        self._perspective_settings = self._get_perspective_settings(self._current_perspective)

        # emit signals
        self.perspective_changed_signal.emit(self._current_perspective.lstrip(self.HIDDEN_PREFIX))
        if settings_changed:
            self.restore_settings_signal.emit(self._global_settings, self._perspective_settings)


    def _get_perspective_settings(self, perspective_name):
        return Settings(self._settings_proxy, 'perspective/%s' % perspective_name)


    def _generate_menu(self):
        create_action = QAction('Create perspective...', self._menu_manager.menu)
        create_action.setIcon(QIcon.fromTheme('list-add'))
        create_action.setIconVisibleInMenu(True)
        create_action.triggered.connect(self._on_create_perspective)
        self._menu_manager.add_suffix(create_action)

        self._remove_action = QAction('Remove perspective...', self._menu_manager.menu)
        self._remove_action.setEnabled(False)
        self._remove_action.setIcon(QIcon.fromTheme('list-remove'))
        self._remove_action.setIconVisibleInMenu(True)
        self._remove_action.triggered.connect(self._on_remove_perspective)
        self._menu_manager.add_suffix(self._remove_action)

        self._menu_manager.add_suffix(None)

        import_action = QAction('Import...', self._menu_manager.menu)
        import_action.setIcon(QIcon.fromTheme('document-open'))
        import_action.setIconVisibleInMenu(True)
        import_action.triggered.connect(self._on_import_perspective)
        self._menu_manager.add_suffix(import_action)

        export_action = QAction('Export...', self._menu_manager.menu)
        export_action.setIcon(QIcon.fromTheme('document-save-as'))
        export_action.setIconVisibleInMenu(True)
        export_action.triggered.connect(self._on_export_perspective)
        self._menu_manager.add_suffix(export_action)

        # add perspectives to menu
        for name in self.perspectives:
            if not name.startswith(self.HIDDEN_PREFIX):
                self._add_perspective_action(name)


    def _on_create_perspective(self):
        name = self._choose_new_perspective_name()
        if name is not None:
            clone_perspective = self._create_perspective_dialog.clone_checkbox.isChecked()
            self._create_perspective(name, clone_perspective)
            self.switch_perspective(name, settings_changed=not clone_perspective, save_before=False)


    def _choose_new_perspective_name(self, show_cloning=True):
        # input dialog for new perspective name
        if self._create_perspective_dialog is None:
            ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'PerspectiveCreate.ui')
            self._create_perspective_dialog = loadUi(ui_file)
            # custom validator preventing forward slashs
            class CustomValidator(QValidator):
                def __init__(self, parent=None):
                    super(CustomValidator, self).__init__(parent)
                def fixup(self, value):
                    value = value.replace('/', '')
                def validate(self, value, pos):
                    if value.find('/') != -1:
                        pos = value.find('/')
                        return (QValidator.Invalid, value, pos)
                    if value == '':
                        return (QValidator.Intermediate, value, pos)
                    return (QValidator.Acceptable, value, pos)
            self._create_perspective_dialog.perspective_name_edit.setValidator(CustomValidator())

        # set default values
        self._create_perspective_dialog.perspective_name_edit.setText('')
        self._create_perspective_dialog.clone_checkbox.setChecked(True)
        self._create_perspective_dialog.clone_checkbox.setVisible(show_cloning)

        # show dialog and wait for it's return value 
        return_value = self._create_perspective_dialog.exec_()
        if return_value == self._create_perspective_dialog.Rejected:
            return

        name = str(self._create_perspective_dialog.perspective_name_edit.text()).lstrip(self.HIDDEN_PREFIX)
        if name == '':
            QMessageBox.warning(self._menu_manager.menu, self.tr('Empty perspective name'), self.tr('The name of the perspective must be non-empty.'))
            return
        if name in self.perspectives:
            QMessageBox.warning(self._menu_manager.menu, self.tr('Duplicate perspective name'), self.tr('A perspective with the same name already exists.'))
            return
        return name

    def _create_perspective(self, name, clone_perspective=True):
        # convert from unicode
        name = str(name)
        if name.find('/') != -1:
            raise RuntimeError('PerspectiveManager._create_perspective() name must not contain forward slashs (/)')

        qDebug('PerspectiveManager._create_perspective(%s, %s)' % (name, clone_perspective))
        # add to list of perspectives
        self.perspectives.append(name)
        self._settings_proxy.set_value('', 'perspectives', self.perspectives)

        # save current settings
        if self._global_settings is not None and self._perspective_settings is not None:
            self.save_settings_signal.emit(self._global_settings, self._perspective_settings)

        # clone settings
        if clone_perspective:
            new_settings = self._get_perspective_settings(name)
            keys = self._perspective_settings.all_keys()
            for key in keys:
                value = self._perspective_settings.value(key)
                new_settings.set_value(key, value)

        # add and switch to perspective
        self._add_perspective_action(name)


    def _add_perspective_action(self, name):
        # create action
        action = QAction(name, self._menu_manager.menu)
        action.setCheckable(True)
        self._perspective_mapper.setMapping(action, name)
        action.triggered.connect(self._perspective_mapper.map)

        # add action to menu
        self._menu_manager.add_item(action)
        # enable remove-action
        if self._menu_manager.count_items() > 1:
            self._remove_action.setEnabled(True)


    def _on_remove_perspective(self):
        # input dialog to choose perspective to be removed
        names = list(self.perspectives)
        names.remove(self._current_perspective)
        name, return_value = QInputDialog.getItem(self._menu_manager.menu, self._menu_manager.tr('Remove perspective'), self._menu_manager.tr('Select the perspective'), names, 0, False)
        # convert from unicode
        name = str(name)
        if return_value == QInputDialog.Rejected:
            return
        if name not in self.perspectives:
            raise UserWarning('unknown perspective: %s' % name)
        qDebug('PerspectiveManager._on_remove_perspective(%s)' % str(name))

        # remove from list of perspectives
        self.perspectives.remove(name)
        self._settings_proxy.set_value('', 'perspectives', self.perspectives)

        # remove settings
        settings = self._get_perspective_settings(name)
        settings.remove('')

        # remove from menu
        self._menu_manager.remove_item(name)

        # disable remove-action
        if self._menu_manager.count_items() < 2:
            self._remove_action.setEnabled(False)


    def _on_import_perspective(self):
        file_name, _ = QFileDialog.getOpenFileName(self._menu_manager.menu, self.tr('Import perspective from file'), None, self.tr('Perspectives (*.perspective)'))
        if file_name is None or file_name == '':
            return

        perspective_name = os.path.basename(file_name)
        suffix = '.perspective'
        if perspective_name.endswith(suffix):
            perspective_name = perspective_name[:-len(suffix)]
        if perspective_name in self.perspectives:
            perspective_name = self._choose_new_perspective_name(False)
            if perspective_name is None:
                return

        self._create_perspective(perspective_name, clone_perspective=False)

        # read perspective from file
        file_handle = open(file_name, 'r')
        #data = eval(file_handle.read())
        data = json.loads(file_handle.read())
        self._convert_values(data, self._import_value)

        new_settings = self._get_perspective_settings(perspective_name)
        new_settings.from_dict(data)

        self.switch_perspective(perspective_name, settings_changed=True, save_before=True)


    def _on_export_perspective(self):
        file_name, _ = QFileDialog.getSaveFileName(self._menu_manager.menu, self.tr('Export perspective to file'), self._current_perspective + '.perspective', self.tr('Perspectives (*.perspective)'))
        if file_name is None or file_name == '':
            return

        # trigger save of perspective before export
        self.save_settings_signal.emit(self._global_settings, self._perspective_settings)

        # convert every value and add pretty print
        data = self._perspective_settings.to_dict()
        self._convert_values(data, self._export_value)

        # write perspective data to file
        file_handle = open(file_name, 'w')
        #file_handle.write(pformat(data))
        file_handle.write(json.dumps(data, indent=2))
        file_handle.close()


    def _convert_values(self, data, convert_function):
        keys = data.get('keys', {})
        for key in keys:
            keys[key] = convert_function(keys[key])
        groups = data.get('groups', {})
        for group in groups:
            self._convert_values(groups[group], convert_function)


    def _import_value(self, value):
        import QtCore #@UnusedImport
        if value['type'] == 'repr':
            return eval(value['repr'])
        elif value['type'] == 'repr(QByteArray.hex)':
            return QByteArray.fromHex(eval(value['repr(QByteArray.hex)']))
        raise RuntimeError('PerspectiveManager._import_value() unknown serialization type (%s)' % value['type'])


    def _export_value(self, value):
        data = {}
        if value.__class__.__name__ == 'QByteArray':
            hex_value = value.toHex()
            data['repr(QByteArray.hex)'] = self._strip_qt_binding_prefix(hex_value, repr(hex_value))
            data['type'] = 'repr(QByteArray.hex)'

            characters = ''
            for i in range(1, value.size(), 2):
                character = value.at(i)
                # output all non-control characters
                if character >= ' ' and character <= '~':
                    characters += character
                else:
                    characters += ' '
            data['pretty-print'] = characters

        else:
            data['repr'] = self._strip_qt_binding_prefix(value, repr(value))
            data['type'] = 'repr'

        # verify that serialized data can be deserialized correctly
        reimported = self._import_value(data)
        if reimported != value:
            raise RuntimeError('PerspectiveManager._export_value() stored value can not be restored (%s)' % type(value))

        return data


    def _strip_qt_binding_prefix(self, obj, data):
        parts = obj.__class__.__module__.split('.')
        if len(parts) > 1 and parts[1] == 'QtCore':
            prefix = '.'.join(parts[:2])
            data = data.replace(prefix, 'QtCore', 1)
        return data


    def _pretty_print(self, obj):
        value = pformat(obj, 2)
        value = self._strip_qt_binding_prefix(obj, value)
        if isinstance(obj, QByteArray):
            value = value.replace('\\x00', '')
            value = re.sub('\\\\x[0-9a-f]{2}', ' ', value)
        return value
