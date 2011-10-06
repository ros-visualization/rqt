from QtBindingHelper import import_from_qt
qDebug, QObject, QSignalMapper, Signal, Slot = import_from_qt(['qDebug', 'QObject', 'QSignalMapper', 'Signal', 'Slot'], 'QtCore')
QAction, QIcon, QInputDialog, QMessageBox = import_from_qt(['QAction', 'QIcon', 'QInputDialog', 'QMessageBox'], 'QtGui')

from MenuManager import MenuManager
from Settings import Settings
from SettingsProxy import SettingsProxy

class PerspectiveManager(QObject):

    perspective_changed_signal = Signal(basestring)
    save_settings_signal = Signal(Settings, Settings)
    restore_settings_signal = Signal(Settings, Settings)

    def __init__(self, settings, menu):
        QObject.__init__(self)
        self.setObjectName('PerspectiveManager')

        self.settings_proxy_ = SettingsProxy(settings)
        self.global_settings_ = Settings(self.settings_proxy_, 'global')
        self.perspective_settings_ = None

        self.menu_manager_ = MenuManager(menu)
        self.perspective_mapper_ = QSignalMapper(menu)
        self.perspective_mapper_.mapped[str].connect(self.switch_perspective)

        # ensure at least one default perspective
        self.perspectives_ = self.global_settings_.value('perspectives/list', [])
        if isinstance(self.perspectives_, basestring):
            self.perspectives_ = [ self.perspectives_ ]
        if not self.perspectives_:
            self.perspectives_.append('Default')
            self.global_settings_.set_value('perspectives/list', self.perspectives_)

        self.current_perspective_ = None
        self.remove_action_ = None

        self.__generate_menu()


    def set_perspective(self, name):
        if name is None:
            current = self.global_settings_.value('perspectives/current')
            name = current if current is not None else 'Default'
        self.switch_perspective(name)


    @Slot(str)
    @Slot(str, bool)
    @Slot(str, bool, bool)
    def switch_perspective(self, name, settings_changed = True, save_before = True):
        if save_before and self.global_settings_ is not None and self.perspective_settings_ is not None:
            self.save_settings_signal.emit(self.global_settings_, self.perspective_settings_)

        # convert from unicode
        name = str(name)

        qDebug('PerspectiveManager.switch_perspective() switching to perspective "%s"' % name)
        if self.current_perspective_ is not None:
            self.menu_manager_.set_item_checked(self.current_perspective_, False)
            self.menu_manager_.set_item_disabled(self.current_perspective_, False)
        if not self.menu_manager_.contains_item(name):
            raise UserWarning('unknown perspective name')

        # update current perspective and emit signal
        self.current_perspective_ = name
        self.menu_manager_.set_item_checked(self.current_perspective_, True)
        self.menu_manager_.set_item_disabled(self.current_perspective_, True)
        self.global_settings_.set_value('perspectives/current', self.current_perspective_)
        self.perspective_settings_ = Settings(self.settings_proxy_, 'perspective/%s' % str(self.current_perspective_))
        self.perspective_changed_signal.emit(self.current_perspective_)
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
            self.__add_perspective(name)


    def __on_create_perspective(self):
        # input dialog for new perspective name
        (name, rc) = QInputDialog.getText(self.menu_manager_.menu(), self.menu_manager_.tr('Create new perspective'), self.menu_manager_.tr('Name of perspective'))
        if not rc:
            qDebug('PerspectiveManager.__on_create_perspective() canceled input dialog for new perspective name')
            return
        if name == '':
            QMessageBox.warning(self.menu_manager_.menu(), self.menu_manager_.tr('Empty perspective name'), self.menu_manager_.tr('The name of the perspective must be non-empty.'))
            return
        if self.menu_manager_.contains_item(name):
            QMessageBox.warning(self.menu_manager_.menu(), self.menu_manager_.tr('Duplicate perspective name'), self.menu_manager_.tr('A perspective with the same name already exists.'))
            return
        qDebug('PerspectiveManager.__on_create_perspective(%s)' % str(name))

        # add to list of perspectives
        self.perspectives_.append(name)
        self.global_settings_.set_value('perspectives/list', self.perspectives_)

        # save current settings
        self.save_settings_signal.emit(self.global_settings_, self.perspective_settings_)
        # clone settings
        new_settings = Settings(self.settings_proxy_, 'perspective/%s' % str(name))
        keys = self.perspective_settings_.all_keys()
        for key in keys:
            value = self.perspective_settings_.value(key)
            new_settings.set_value(key, value)

        # add and switch to perspective
        self.__add_perspective(name)
        self.switch_perspective(name, False, False)


    def __on_remove_perspective(self):
        # input dialog to choose perspective to be removed
        names = self.menu_manager_.get_items()
        names.remove(self.current_perspective_)
        (name, rc) = QInputDialog.getItem(self.menu_manager_.menu(), self.menu_manager_.tr('Remove perspective'), self.menu_manager_.tr('Select the perspective'), names, 0, False)
        if not rc:
            qDebug('PerspectiveManager.__on_remove_perspective() canceled input dialog for perspective name')
            return
        if not self.menu_manager_.contains_item(name):
            raise UserWarning('unknown perspective name')
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


    def __add_perspective(self, name):
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
