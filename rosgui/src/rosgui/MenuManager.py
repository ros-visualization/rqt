import QtBindingHelper #@UnusedImport
from QtCore import QObject
from QtGui import QAction, QMenu

class MenuManager(QObject):

    def __init__(self, menu):
        super(MenuManager, self).__init__()
        self.setObjectName('MenuManager')

        self.menu_ = menu
        self.prefixes_ = []
        self.prefixes_separator_ = None
        self.ordered_items_ = []
        self.suffixes_ = []
        self.suffixes_separator_ = None

        # get already existing items from menu
        for action in menu.actions():
            menu = action.menu()
            if menu is not None:
                self.ordered_items_.append(menu)
            else:
                self.ordered_items_.append(action)

    def menu(self):
        return self.menu_

    def add_prefix(self, item):
        if self.prefixes_separator_ is None:
            before = self.ordered_items_[0] if self.ordered_items_ else None
            if isinstance(before, QMenu):
                before = before.menuAction()
            self.prefixes_separator_ = self.menu_.insertSeparator(before)
        self.__insert_item(self.prefixes_separator_, item)
        self.prefixes_.append(item)

    def add_item(self, new_item):
        for i, item in enumerate(self.ordered_items_):
            if self.__get_item_label(item) > self.__get_item_label(new_item):
                self.__insert_item(item, new_item)
                self.ordered_items_.insert(i, new_item)
                return

        before = self.suffixes_separator_ if self.suffixes_separator_ is not None else None
        self.__insert_item(before, new_item)
        self.ordered_items_.append(new_item)

    def add_suffix(self, item):
        if self.suffixes_separator_ is None:
            self.suffixes_separator_ = self.menu_.addSeparator()
        self.__insert_item(None, item)
        self.suffixes_.append(item)

    def count_items(self):
        return len(self.ordered_items_)

    def contains_item(self, name):
        return self.get_item(name) is not None

    def get_item(self, name):
        if isinstance(name, QAction) or isinstance(name, QMenu):
            if name in self.ordered_items_:
                return name
            return None
        for item in self.ordered_items_:
            if self.__get_item_label(item) == name:
                return item
        return None

    def contains_menu(self, name):
        return self.get_menu(name) is not None

    def get_menu(self, name):
        for item in self.ordered_items_:
            if self.__get_item_label(item) == name and isinstance(item, QMenu):
                return item
        return None

    def get_items(self):
        names = []
        for item in self.ordered_items_:
            names.append(self.__get_item_label(item))
        return names

    def set_item_checked(self, name, flag):
        item = self.get_item(name)
        if item is not None:
            item.setChecked(flag)

    def set_item_disabled(self, name, flag):
        item = self.get_item(name)
        if item is not None:
            item.setDisabled(flag)

    def remove_item(self, name):
        item = self.get_item(name)
        if item is not None:
            self.menu_.removeAction(item)
            self.ordered_items_.remove(item)
            item.deleteLater()

    def __insert_item(self, before, item):
        if isinstance(before, QMenu):
            before = before.menuAction()
        if isinstance(item, QAction):
            self.menu_.insertAction(before, item)
        elif isinstance(item, QMenu):
            self.menu_.insertMenu(before, item)
        else:
            raise UserWarning('unknown item type')

    def __get_item_label(self, item):
        if isinstance(item, QAction):
            return item.text()
        elif isinstance(item, QMenu):
            return item.title()
        else:
            raise UserWarning('unknown item type')
