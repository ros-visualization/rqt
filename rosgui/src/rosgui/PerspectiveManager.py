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

        self.settings_proxy_ = SettingsProxy(settings)
        self.global_settings_ = Settings(self.settings_proxy_, 'global')
        self.perspective_settings_ = None
        self.create_perspective_dialog = None

        self.menu_manager_ = MenuManager(menu)
        self.perspective_mapper_ = QSignalMapper(menu)
        self.perspective_mapper_.mapped[str].connect(self.switch_perspective)

        # get perspective list from settings
        self.perspectives_ = self.settings_proxy_.value('', 'perspectives', [])
        if isinstance(self.perspectives_, basestring):
            self.perspectives_ = [ self.perspectives_ ]

        self.current_perspective_ = None
        self.remove_action_ = None

        self.__generate_menu()


    def set_perspective(self, name, hide_perspective=False):
        if name is None:
            name = self.settings_proxy_.value('', 'current-perspective', 'Default')
        elif hide_perspective:
            name = self.HIDDEN_PREFIX + name
        self.switch_perspective(name)


    @Slot(str)
    @Slot(str, bool)
    @Slot(str, bool, bool)
    def switch_perspective(self, name, settings_changed=True, save_before=True):
        if save_before and self.global_settings_ is not None and self.perspective_settings_ is not None:
            self.save_settings_signal.emit(self.global_settings_, self.perspective_settings_)

        # convert from unicode
        name = str(name)

        qDebug('PerspectiveManager.switch_perspective() switching to perspective "%s"' % name)
        if self.current_perspective_ is not None:
            self.menu_manager_.set_item_checked(self.current_perspective_, False)
            self.menu_manager_.set_item_disabled(self.current_perspective_, False)

        # update current perspective
        self.current_perspective_ = name
        self.menu_manager_.set_item_checked(self.current_perspective_, True)
        self.menu_manager_.set_item_disabled(self.current_perspective_, True)
        if not self.current_perspective_.startswith(self.HIDDEN_PREFIX):
            self.settings_proxy_.set_value('', 'current-perspective', self.current_perspective_)
        self.perspective_settings_ = self._get_perspective_settings(self.current_perspective_)

        # create perspective if necessary 
        if name not in self.perspectives_:
            qDebug('PerspectiveManager.switch_perspective(): unknown perspective %s' % name)
            self.__create_perspective(name, clone_perspective=False)
            self.switch_perspective(name, settings_changed=True, save_before=False)
            return

        # emit signals
        self.perspective_changed_signal.emit(self.current_perspective_.lstrip(self.HIDDEN_PREFIX))
        if settings_changed:
            self.restore_settings_signal.emit(self.global_settings_, self.perspective_settings_)


    def _get_perspective_settings(self, perspective_name):
        return Settings(self.settings_proxy_, 'perspective/%s' % perspective_name)


    def __generate_menu(self):
        create_action = QAction('Create perspective...', self.menu_manager_.menu())
        create_action.setIcon(QIcon.fromTheme('list-add'))
        create_action.setIconVisibleInMenu(True)
        create_action.triggered.connect(self._on_create_perspective)
        self.menu_manager_.add_suffix(create_action)

        self.remove_action_ = QAction('Remove perspective...', self.menu_manager_.menu())
        self.remove_action_.setEnabled(False)
        self.remove_action_.setIcon(QIcon.fromTheme('list-remove'))
        self.remove_action_.setIconVisibleInMenu(True)
        self.remove_action_.triggered.connect(self._on_remove_perspective)
        self.menu_manager_.add_suffix(self.remove_action_)

        self.menu_manager_.add_suffix(None)

        import_action = QAction('Import...', self.menu_manager_.menu())
        import_action.setIcon(QIcon.fromTheme('document-open'))
        import_action.setIconVisibleInMenu(True)
        import_action.triggered.connect(self._on_import_perspective)
        self.menu_manager_.add_suffix(import_action)

        export_action = QAction('Export...', self.menu_manager_.menu())
        export_action.setIcon(QIcon.fromTheme('document-save-as'))
        export_action.setIconVisibleInMenu(True)
        export_action.triggered.connect(self._on_export_perspective)
        self.menu_manager_.add_suffix(export_action)

        # add perspectives to menu
        for name in self.perspectives_:
            if not name.startswith(self.HIDDEN_PREFIX):
                self.__add_perspective_action(name)


    def _on_create_perspective(self):
        name = self.__choose_new_perspective_name()
        if name is not None:
            clone_perspective = self.create_perspective_dialog.clone_checkbox.isChecked()
            self.__create_perspective(name, clone_perspective)
            self.switch_perspective(name, settings_changed=not clone_perspective, save_before=False)


    def __choose_new_perspective_name(self, show_cloning=True):
        # input dialog for new perspective name
        if self.create_perspective_dialog is None:
            ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'PerspectiveCreate.ui')
            self.create_perspective_dialog = loadUi(ui_file)
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
            self.create_perspective_dialog.perspective_name_edit.setValidator(CustomValidator())

        # set default values
        self.create_perspective_dialog.perspective_name_edit.setText('')
        self.create_perspective_dialog.clone_checkbox.setChecked(True)
        self.create_perspective_dialog.clone_checkbox.setVisible(show_cloning)

        # show dialog and wait for it's return value 
        return_value = self.create_perspective_dialog.exec_()
        if return_value == self.create_perspective_dialog.Rejected:
            return

        name = str(self.create_perspective_dialog.perspective_name_edit.text()).lstrip(self.HIDDEN_PREFIX)
        if name == '':
            QMessageBox.warning(self.menu_manager_.menu(), self.tr('Empty perspective name'), self.tr('The name of the perspective must be non-empty.'))
            return
        if name in self.perspectives_:
            QMessageBox.warning(self.menu_manager_.menu(), self.tr('Duplicate perspective name'), self.tr('A perspective with the same name already exists.'))
            return
        return name

    def __create_perspective(self, name, clone_perspective=True):
        # convert from unicode
        name = str(name)
        if name.find('/') != -1:
            raise RuntimeError('PerspectiveManager.__create_perspective() name must not contain forward slashs (/)')

        qDebug('PerspectiveManager.__create_perspective(%s, %s)' % (name, clone_perspective))
        # add to list of perspectives
        self.perspectives_.append(name)
        self.settings_proxy_.set_value('', 'perspectives', self.perspectives_)

        # save current settings
        if self.global_settings_ is not None and self.perspective_settings_ is not None:
            self.save_settings_signal.emit(self.global_settings_, self.perspective_settings_)

        # clone settings
        new_settings = self._get_perspective_settings(name)
        if clone_perspective:
            keys = self.perspective_settings_.all_keys()
            for key in keys:
                value = self.perspective_settings_.value(key)
                new_settings.set_value(key, value)

        # add and switch to perspective
        self.__add_perspective_action(name)


    def __add_perspective_action(self, name):
        # create action
        action = QAction(name, self.menu_manager_.menu())
        action.setCheckable(True)
        self.perspective_mapper_.setMapping(action, name)
        action.triggered.connect(self.perspective_mapper_.map)

        # add action to menu
        self.menu_manager_.add_item(action)
        # enable remove-action
        if self.menu_manager_.count_items() > 1:
            self.remove_action_.setEnabled(True)


    def _on_remove_perspective(self):
        # input dialog to choose perspective to be removed
        names = list(self.perspectives_)
        names.remove(self.current_perspective_)
        name, return_value = QInputDialog.getItem(self.menu_manager_.menu(), self.menu_manager_.tr('Remove perspective'), self.menu_manager_.tr('Select the perspective'), names, 0, False)
        # convert from unicode
        name = str(name)
        if return_value == QInputDialog.Rejected:
            return
        if name not in self.perspectives_:
            raise UserWarning('unknown perspective: %s' % name)
        qDebug('PerspectiveManager._on_remove_perspective(%s)' % str(name))

        # remove from list of perspectives
        self.perspectives_.remove(name)
        self.settings_proxy_.set_value('', 'perspectives', self.perspectives_)

        # remove settings
        settings = self._get_perspective_settings(name)
        settings.remove('')

        # remove from menu
        self.menu_manager_.remove_item(name)

        # disable remove-action
        if self.menu_manager_.count_items() < 2:
            self.remove_action_.setEnabled(False)


    def _on_import_perspective(self):
        file_name = QFileDialog.getOpenFileName(self.menu_manager_.menu(), self.tr('Import perspective from file'), None, self.tr('Perspectives (*.perspective)'))
        if file_name is None or file_name == '':
            return

        perspective_name = os.path.basename(file_name)
        suffix = '.perspective'
        if perspective_name.endswith(suffix):
            perspective_name = perspective_name[:-len(suffix)]
        if perspective_name in self.perspectives_:
            perspective_name = self.__choose_new_perspective_name(False)
            if perspective_name is None:
                return

        self.__create_perspective(perspective_name, clone_perspective=False)

        # read perspective from file
        file_handle = open(file_name, 'r')
        #data = eval(file_handle.read())
        data = json.loads(file_handle.read())
        self.__convert_values(data, self.__import_value)

        new_settings = self._get_perspective_settings(perspective_name)
        new_settings.from_dict(data)

        self.switch_perspective(perspective_name, settings_changed=True, save_before=True)


    def _on_export_perspective(self):
        file_name = QFileDialog.getSaveFileName (self.menu_manager_.menu(), self.tr('Export perspective to file'), self.current_perspective_ + '.perspective', self.tr('Perspectives (*.perspective)'))
        if file_name is None or file_name == '':
            return

        # trigger save of perspective before export
        self.save_settings_signal.emit(self.global_settings_, self.perspective_settings_)

        # convert every value and add pretty print
        data = self.perspective_settings_.to_dict()
        self.__convert_values(data, self.__export_value)

        # write perspective data to file
        file_handle = open(file_name, 'w')
        #file_handle.write(pformat(data))
        file_handle.write(json.dumps(data, indent=2))
        file_handle.close()


    def __convert_values(self, data, convert_function):
        keys = data.get('keys', {})
        for key in keys:
            keys[key] = convert_function(keys[key])
        groups = data.get('groups', {})
        for group in groups:
            self.__convert_values(groups[group], convert_function)


    def __import_value(self, value):
        import QtCore #@UnusedImport
        return eval(value['repr'])


    def __export_value(self, value):
        mod_repr = self.__strip_qt_binding_prefix(value, repr(value))
        pretty_print = self.__pretty_print(value)
        data = {
            'repr': mod_repr,
            'pretty-print': pretty_print,
        }
        reimported = self.__import_value(data)
        if reimported != value:
            raise RuntimeError('PerspectiveManager.__export_value() stored value can not be restored (%s)' % type(value))
        return data


    def __strip_qt_binding_prefix(self, obj, data):
        if hasattr(obj, '__module__'):
            parts = obj.__module__.split('.')
            if len(parts) > 1 and parts[1] == 'QtCore':
                prefix = '.'.join(parts[:2])
                data = data.replace(prefix, 'QtCore', 1)
        return data


    def __pretty_print(self, obj):
        value = pformat(obj, 2)
        value = self.__strip_qt_binding_prefix(obj, value)
        if isinstance(obj, QByteArray):
            value = value.replace('\\x00', '')
            value = re.sub('\\\\x[0-9a-f]{2}', ' ', value)
        return value
