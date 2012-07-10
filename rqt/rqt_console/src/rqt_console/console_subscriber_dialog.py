import os

import qt_gui.qt_binding_helper  # @UnusedImport
from QtGui import QDialog
from QtCore import qWarning
from qt_gui.qt_binding_helper import loadUi

class ConsoleSubscriberDialog(QDialog):
    def __init__(self, caller, topics):
        super(QDialog, self).__init__()
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'console_subscriber_dialog.ui')
        loadUi(ui_file, self)

        topics.sort(reverse=True)
        for topic in topics:
            self.topic_combo.addItem(topic[0] + ' (' + topic[1] + ')')

        self._caller = caller

        # captures current list level needed for the first calls since "currentItem" is not set 
        # when programatically calling a callback from a callback
        self._node_index = -1  
        self._logger_index = -1  
        self._level_index = -1  
        self.node_list.currentRowChanged[int].connect(self.node_changed)
        self.logger_list.currentRowChanged[int].connect(self.logger_changed)
        self.level_list.currentRowChanged[int].connect(self.level_changed)
        self.topic_combo.activated[int].connect(self.topic_changed)
        self.refresh_button.clicked[bool].connect(self.refresh_nodes)

        self.refresh_nodes()
        if self.node_list.count() > 0:
            self.node_list.item(0).setSelected(True)
            self.node_changed(0)

    def refresh_nodes(self):
        self.level_list.clear()
        self.logger_list.clear()
        self.node_list.clear()
        for name in self._caller.get_node_names():
            self.node_list.addItem(name)

    def topic_changed(self, row):
        index = self.topic_combo.itemText(row).find(' (')
        if index is not -1:
            self._caller.subscribe_topic(self.topic_combo.itemText(row)[:index])

    def node_changed(self, row):
        if row == -1:
            return
        if row < 0 or row >= self.node_list.count():
            qWarning(self.tr('Node row %s out of bounds. Current count: %s' % (row,self.node_list.count())))
            return
        self._node_index = row
        self.logger_list.clear()
        self.level_list.clear()
        loggers = self._caller.get_loggers(self.node_list.item(row).text())
        if len(loggers) is 0:
            return
        for logger in sorted(loggers):
            self.logger_list.addItem(logger)
        if self.logger_list.count() != 0:
            self.logger_list.setCurrentRow(0)

    def logger_changed(self, row):
        if row == -1:
            return
        if row < 0 or row >= self.logger_list.count():
            qWarning(self.tr('Logger row %s out of bounds. Current count: %s' % (row,self.logger_list.count())))
            return
        self._logger_index = row
        
        if self.level_list.count() == 0:
            for level in self._caller.get_levels():
                self.level_list.addItem(level)
        for index in range(self.level_list.count()):
            if self.level_list.item(index).text().lower() == self._caller._current_levels[self.logger_list.item(row).text()].lower():
                self.level_list.setCurrentRow(index)

    def level_changed(self, row):
        if row == -1:
            return
        if row < 0 or row >= self.level_list.count():
            qWarning(self.tr('Level row %s out of bounds. Current count: %s' % (row,self.level_list.count())))
            return
        self._level_index = row
        self._caller.send_logger_change_message(self.node_list.item(self._node_index).text(), self.logger_list.item(self._logger_index).text(), self.level_list.item(row).text())

