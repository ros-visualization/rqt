import os

from QtBindingHelper import loadUi
from QtCore import qDebug, QObject, QSignalMapper, Signal, Slot
from QtGui import QAction, QIcon, QInputDialog, QMessageBox

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
        self.perspectives_ = self.global_settings_.value('perspectives/list', [])
        if isinstance(self.perspectives_, basestring):
            self.perspectives_ = [ self.perspectives_ ]

        self.current_perspective_ = None
        self.remove_action_ = None

        self.__generate_menu()


    def set_perspective(self, name, hide_perspective=False):
        if name is None:
            current = self.global_settings_.value('perspectives/current')
            name = current if current is not None else 'Default'
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
            self.global_settings_.set_value('perspectives/current', self.current_perspective_)
        self.perspective_settings_ = Settings(self.settings_proxy_, 'perspective/%s' % self.current_perspective_)

        # create perspective if necessary 
        if name not in self.perspectives_:
            qDebug('PerspectiveManager.switch_perspective(): unknown perspective %s' % name)
            self.__create_perspective(name, clone_perspective=False)
            return

        # emit signals
        self.perspective_changed_signal.emit(self.current_perspective_.lstrip(self.HIDDEN_PREFIX))
        if settings_changed:
            self.restore_settings_signal.emit(self.global_settings_, self.perspective_settings_)


    def __generate_menu(self):
        create_action = QAction('Create perspective...', self.menu_manager_.menu())
        create_action.setIcon(QIcon.fromTheme('list-add'))
        create_action.setIconVisibleInMenu(True)
        create_action.triggered.connect(self.__on_create_perspective)
        self.menu_manager_.add_suffix(create_action)

        self.remove_action_ = QAction('Remove perspective...', self.menu_manager_.menu())
        self.remove_action_.setEnabled(False)
        self.remove_action_.setIcon(QIcon.fromTheme('list-remove'))
        self.remove_action_.setIconVisibleInMenu(True)
        self.remove_action_.triggered.connect(self.__on_remove_perspective)
        self.menu_manager_.add_suffix(self.remove_action_)

        # add perspectives to menu
        for name in self.perspectives_:
            if not name.startswith(self.HIDDEN_PREFIX):
                self.__add_perspective_action(name)


    def __on_create_perspective(self):
        # input dialog for new perspective name
        if self.create_perspective_dialog is None:
            ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'PerspectiveCreate.ui')
            self.create_perspective_dialog = loadUi(ui_file)

        # set default values
        self.create_perspective_dialog.perspective_name_edit.setText('')
        self.create_perspective_dialog.clone_checkbox.setChecked(True)

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

        clone_perspective = self.create_perspective_dialog.clone_checkbox.isChecked()
        self.__create_perspective(name, clone_perspective)


    def __create_perspective(self, name, clone_perspective=True):
        qDebug('PerspectiveManager.__create_perspective(%s, %s)' % (name, clone_perspective))
        # add to list of perspectives
        self.perspectives_.append(name)
        self.global_settings_.set_value('perspectives/list', self.perspectives_)

        # save current settings
        if self.global_settings_ is not None and self.perspective_settings_ is not None:
            self.save_settings_signal.emit(self.global_settings_, self.perspective_settings_)

        # clone settings
        new_settings = Settings(self.settings_proxy_, 'perspective/%s' % str(name))
        if clone_perspective:
            keys = self.perspective_settings_.all_keys()
            for key in keys:
                value = self.perspective_settings_.value(key)
                new_settings.set_value(key, value)

        # add and switch to perspective
        self.__add_perspective_action(name)
        self.switch_perspective(name, settings_changed=not clone_perspective, save_before=False)


    def __on_remove_perspective(self):
        # input dialog to choose perspective to be removed
        names = list(self.perspectives_)
        names.remove(self.current_perspective_)
        name, return_value = QInputDialog.getItem(self.menu_manager_.menu(), self.menu_manager_.tr('Remove perspective'), self.menu_manager_.tr('Select the perspective'), names, 0, False)
        if return_value == QInputDialog.Rejected:
            return
        if name not in self.perspectives_:
            raise UserWarning('unknown perspective: %s' % name)
        qDebug('PerspectiveManager.__on_remove_perspective(%s)' % str(name))

        # remove from list of perspectives
        self.perspectives_.remove(name)
        self.global_settings_.set_value('perspectives/list', self.perspectives_)

        # remove settings
        settings = Settings(self.settings_proxy_, 'perspective/%s' % str(name))
        settings.remove('')

        # remove from menu
        self.menu_manager_.remove_item(name)

        # disable remove-action
        if self.menu_manager_.count_items() < 2:
            self.remove_action_.setEnabled(False)


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
