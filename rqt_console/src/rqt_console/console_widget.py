# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import os

from python_qt_binding import loadUi
from python_qt_binding.QtGui import QApplication, QCursor, QFileDialog, QIcon, QMenu, QMessageBox, QTableView, QWidget
from python_qt_binding.QtCore import QRegExp, Qt, qWarning

from .filters.custom_filter import CustomFilter
from .filters.location_filter import LocationFilter
from .filters.message_filter import MessageFilter
from .filters.node_filter import NodeFilter
from .filters.severity_filter import SeverityFilter
from .filters.time_filter import TimeFilter
from .filters.topic_filter import TopicFilter

from .filters.custom_filter_widget import CustomFilterWidget
from .filters.filter_wrapper_widget import FilterWrapperWidget
from .filters.list_filter_widget import ListFilterWidget
from .filters.text_filter_widget import TextFilterWidget
from .filters.time_filter_widget import TimeFilterWidget

from .text_browse_dialog import TextBrowseDialog


class ConsoleWidget(QWidget):
    """
    Primary widget for the rqt_console plugin.
    """
    def __init__(self, proxymodel, minimal = False):
        """
        :param proxymodel: the proxy model to display in the widget,''QSortFilterProxyModel''
        :param minimal: if true the load, save and column buttons will be hidden as well as the filter splitter, ''bool''
        """
        super(ConsoleWidget, self).__init__()
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'console_widget.ui')
        loadUi(ui_file, self)
        
        if minimal:
            self.load_button.hide()
            self.save_button.hide()
            self.column_resize_button.hide()
            self.filter_splitter.hide()
        self.setObjectName('ConsoleWidget')
        self.table_view.setModel(proxymodel)
        self._proxymodel = proxymodel
        self._datamodel = proxymodel.sourceModel()
                
        self._columnwidth = (600, 140, 200, 430, 200, 600)
        for idx, width in enumerate(self._columnwidth):
            self.table_view.horizontalHeader().resizeSection(idx, width)
        self.table_view.sortByColumn(3, Qt.DescendingOrder)

        self.add_exclude_button.setIcon(QIcon.fromTheme('list-add'))
        self.add_highlight_button.setIcon(QIcon.fromTheme('list-add'))
        self._pauseicon = QIcon.fromTheme('media-playback-pause')
        self._recordicon = QIcon.fromTheme('media-record')
        self.pause_button.setIcon(self._pauseicon)
        self.load_button.setIcon(QIcon.fromTheme('document-open'))
        self.save_button.setIcon(QIcon.fromTheme('document-save'))
        self.highlight_exclude_button.setIcon(QIcon.fromTheme('format-text-strikethrough'))

        self.pause_button.clicked[bool].connect(self._handle_pause_clicked)
        self.load_button.clicked[bool].connect(self._handle_load_clicked)
        self.save_button.clicked[bool].connect(self._handle_save_clicked)
        self.column_resize_button.clicked[bool].connect(self._handle_column_resize_clicked)

        self.table_view.mouseDoubleClickEvent = self._handle_mouse_double_click
        self.table_view.mousePressEvent = self._handle_mouse_press
        self.table_view.keyPressEvent = self._handle_custom_keypress
        self.severitylist = self._datamodel.get_severity_list()

        # These are lists of Tuples = (,)
        self._exclude_filters = []
        self._highlight_filters = []

        self.highlight_exclude_button.clicked[bool].connect(self._proxymodel.set_show_highlighted_only)

        self.add_highlight_button.clicked.connect(self._add_highlight_filter)
        self.add_exclude_button.clicked.connect(self._add_exclude_filter)

        # Filter factory dictionary:
        # index 0 is a label describing the widget, index 1 is the class that provides filtering logic
        # index 2 is the widget that sets the data in the filter class, index 3 are the arguments for the widget class constructor
        self.filter_factory = {'message': (self.tr('Message Filter'), MessageFilter, TextFilterWidget, []),
                               'severity': (self.tr('Severity Filter'), SeverityFilter, ListFilterWidget, [self._datamodel.get_severity_list]),
                               'node': (self.tr('Node Filter'), NodeFilter, ListFilterWidget, [self._datamodel.get_unique_col_data, 2]),
                               'time': (self.tr('Time Filter'), TimeFilter, TimeFilterWidget, [self.get_time_range_from_selection]),
                               'topic': (self.tr('Topic Filter'), TopicFilter, ListFilterWidget, [self._datamodel.get_unique_col_data, 4]),
                               'location': (self.tr('Location Filter'), LocationFilter, TextFilterWidget, []),
                               'custom': (self.tr('Custom Filter'), CustomFilter, CustomFilterWidget, [self._datamodel.get_severity_list, self._datamodel.get_unique_col_data, 2, self._datamodel.get_unique_col_data, 4])}

        # list of TextBrowserDialogs to close when cleaning up
        self._browsers = []

        # This defaults the filters panel to start by taking 50% of the available space
        self.table_splitter.setSizes([1, 1])
        self.exclude_table.resizeColumnsToContents()
        self.highlight_table.resizeColumnsToContents()

    def get_time_range_from_selection(self):
        """
        :returns: the range of time of messages in the current table selection (min, max), ''tuple(str,str)''
        """
        rowlist = []
        indexes = self.table_view.selectionModel().selectedIndexes()

        if len(indexes) != 0:
            for current in indexes:
                rowlist.append(self._proxymodel.mapToSource(current).row())
            rowlist = list(set(rowlist))
            rowlist.sort()

            mintime, maxtime = self._datamodel.get_time_range(rowlist)
            return (mintime, maxtime)
        return (-1, -1)

    def _delete_highlight_filter(self):
        """
        Deletes any highlight filters which have a checked delete button
        """
        for index, item in enumerate(self._highlight_filters):
            if item[1].delete_button.isChecked():
                if self._proxymodel.delete_highlight_filter(index):
                    self.highlight_table.removeCellWidget(index, 0)
                    self.highlight_table.removeRow(index)
                    item[0].filter_changed_signal.disconnect(self._proxymodel.handle_filters_changed)
                    item[1].delete_button.clicked.disconnect(self._delete_highlight_filter)
                    del self._highlight_filters[index]

    def _delete_exclude_filter(self):
        """
        Deletes any exclude filters which have a checked delete button
        """
        for index, item in enumerate(self._exclude_filters):
            if item[1].delete_button.isChecked():
                if self._proxymodel.delete_exclude_filter(index):
                    self.exclude_table.removeCellWidget(index, 0)
                    self.exclude_table.removeRow(index)
                    item[0].filter_changed_signal.disconnect(self._proxymodel.handle_filters_changed)
                    item[1].delete_button.clicked.disconnect(self._delete_exclude_filter)
                    del self._exclude_filters[index]

    def _add_highlight_filter(self, filter_index=False):
        """
        :param filter_index: if false then this function shows a QMenu to allow the user to choose a type of message filter. ''bool''
        OR
        :param filter_index: the index of the filter to be added, ''int''
        :return: if a filter was added then the index is returned, ''int''
        OR
        :return: if no filter was added then None is returned, ''NoneType''
        """
        if filter_index is False:
            filter_index = -1
            filter_select_menu = QMenu()
            for index, item in self.filter_factory.items():
                # flattens the _highlight filters list and only adds the item if it doesn't already exist
                if self.filter_factory[index][0] == self.tr('Message Filter') or not self.filter_factory[index][1] in [type(item) for sublist in self._highlight_filters for item in sublist]:
                    filter_select_menu.addAction(self.filter_factory[index][0])
            action = filter_select_menu.exec_(QCursor.pos())
            if action is None:
                return
            for index, item in self.filter_factory.items():
                if self.filter_factory[index][0] == action.text():
                    filter_index = index
            if filter_index == -1:
                return

        index = len(self._highlight_filters)
        newfilter = self.filter_factory[filter_index][1]()
        newwidget = self.filter_factory[filter_index][2](newfilter, self.filter_factory[filter_index][3])

        # pack the new filter tuple onto the filter list
        self._highlight_filters.append((newfilter, FilterWrapperWidget(newwidget, self.filter_factory[filter_index][0]), filter_index))
        self._proxymodel.add_highlight_filter(newfilter)
        newfilter.filter_changed_signal.connect(self._proxymodel.handle_filters_changed)
        self._highlight_filters[index][1].delete_button.clicked.connect(self._delete_highlight_filter)
        self._datamodel.rowsInserted.connect(self._highlight_filters[index][1].repopulate)

        # place the widget in the proper location
        self.highlight_table.insertRow(index)
        self.highlight_table.setCellWidget(index, 0, self._highlight_filters[index][1])
        self.highlight_table.resizeColumnsToContents()
        self.highlight_table.resizeRowsToContents()
        newfilter.filter_changed_signal.emit()
        return index

    def _add_exclude_filter(self, filter_index=False):
        """
        :param filter_index: if false then this function shows a QMenu to allow the user to choose a type of message filter. ''bool''
        OR
        :param filter_index: the index of the filter to be added, ''int''
        :return: if a filter was added then the index is returned, ''int''
        OR
        :return: if no filter was added then None is returned, ''NoneType''
        """
        if filter_index is False:
            filter_index = -1
            filter_select_menu = QMenu()
            for index, item in self.filter_factory.items():
                # flattens the _exclude filters list and only adds the item if it doesn't already exist
                if self.filter_factory[index][0] == self.tr('Message Filter') or not self.filter_factory[index][1] in [type(item) for sublist in self._exclude_filters for item in sublist]:
                    filter_select_menu.addAction(self.filter_factory[index][0])
            action = filter_select_menu.exec_(QCursor.pos())
            if action is None:
                return None
            for index, item in self.filter_factory.items():
                if self.filter_factory[index][0] == action.text():
                    filter_index = index
            if filter_index == -1:
                return None

        index = len(self._exclude_filters)
        newfilter = self.filter_factory[filter_index][1]()
        newwidget = self.filter_factory[filter_index][2](newfilter, self.filter_factory[filter_index][3])

        # pack the new filter tuple onto the filter list
        self._exclude_filters.append((newfilter, FilterWrapperWidget(newwidget, self.filter_factory[filter_index][0]), filter_index))
        self._proxymodel.add_exclude_filter(newfilter)
        newfilter.filter_changed_signal.connect(self._proxymodel.handle_filters_changed)
        self._exclude_filters[index][1].delete_button.clicked.connect(self._delete_exclude_filter)
        self._datamodel.rowsInserted.connect(self._exclude_filters[index][1].repopulate)

        # place the widget in the proper location
        self.exclude_table.insertRow(index)
        self.exclude_table.setCellWidget(index, 0, self._exclude_filters[index][1])
        self.exclude_table.resizeColumnsToContents()
        self.exclude_table.resizeRowsToContents()
        newfilter.filter_changed_signal.emit()
        return index

    def _process_highlight_exclude_filter(self, selection, selectiontype, exclude=False):
        """
        Modifies the relevant filters (based on selectiontype) to remove (exclude=True)
        or highlight (exclude=False) the selection from the dataset in the tableview.
        :param selection: the actual selection, ''str''
        :param selectiontype: the type of selection, ''str''
        :param exclude: If True process as an exclude filter, False process as an highlight filter, ''bool''
        """
        types = {self.tr('Node'): 2, self.tr('Topic'): 4, self.tr('Severity'): 1, self.tr('Message'): 0}
        try:
            col = types[selectiontype]
        except:
            raise RuntimeError("Bad Column name in ConsoleWidget._process_highlight_exclude_filter()")

        if col == 0:
            unique_messages = set()
            selected_indexes = self.table_view.selectionModel().selectedIndexes()
            num_selected = len(selected_indexes) / 6
            for index in range(num_selected):
                unique_messages.add(selected_indexes[num_selected * col + index].data())
            unique_messages = list(unique_messages)
            for message in unique_messages:
                message = message.replace('\\', '\\\\')
                message = message.replace('.', '\\.')
                if exclude:
                    filter_index = self._add_exclude_filter(selectiontype.lower())
                    filter_widget = self._exclude_filters[filter_index][1].findChildren(QWidget, QRegExp('.*FilterWidget.*'))[0]
                    filter_widget.set_regex(True)
                    filter_widget.set_text('^' + message + '$')
                else:
                    filter_index = self._add_highlight_filter(col)
                    filter_widget = self._highlight_filters[filter_index][1].findChildren(QWidget, QRegExp('.*FilterWidget.*'))[0]
                    filter_widget.set_regex(True)
                    filter_widget.set_text('^' + message + '$')

        else:
            if exclude:
                # Test if the filter we are adding already exists if it does use the existing filter
                if self.filter_factory[selectiontype.lower()][1] not in [type(item) for sublist in self._exclude_filters for item in sublist]:
                    filter_index = self._add_exclude_filter(selectiontype.lower())
                else:
                    for index, item in enumerate(self._exclude_filters):
                        if type(item[0]) == self.filter_factory[selectiontype.lower()][1]:
                            filter_index = index
            else:
                # Test if the filter we are adding already exists if it does use the existing filter
                if self.filter_factory[selectiontype.lower()][1] not in [type(item) for sublist in self._highlight_filters for item in sublist]:
                    filter_index = self._add_highlight_filter(col)
                else:
                    for index, item in enumerate(self._highlight_filters):
                        if type(item[0]) == self.filter_factory[selectiontype.lower()][1]:
                            filter_index = index

            if exclude:
                filter_widget = self._exclude_filters[filter_index][1].findChildren(QWidget, QRegExp('.*FilterWidget.*'))[0]
                filter_widget.select_item(selection)
            else:
                filter_widget = self._highlight_filters[filter_index][1].findChildren(QWidget, QRegExp('.*FilterWidget.*'))[0]
                filter_widget.select_item(selection)

    def _rightclick_menu(self, event):
        """
        Dynamically builds the rightclick menu based on the unique column data
        from the passed in datamodel and then launches it modally
        :param event: the mouse event object, ''QMouseEvent''
        """
        severities = self._datamodel.get_unique_col_data(1)
        nodes = self._datamodel.get_unique_col_data(2)
        topics = self._datamodel.get_unique_col_data(4)
        temp = []
        for topic in topics:
            if topic.find(', ') == -1:
                temp.append(topic)
            else:
                temp = temp + topic.split(', ')
        topics = list(set(temp))

        columns = list(self._datamodel.message_members())
        for index in range(len(columns)):
            columns[index] = [columns[index][1:].capitalize()]

        # menutext entries turned into
        menutext = []
        menutext.append([self.tr('Exclude'), [[self.tr('Severity'), severities], [self.tr('Node'), nodes], [self.tr('Topic'), topics], [self.tr('Selected Message(s)')]]])
        menutext.append([self.tr('Highlight'), [[self.tr('Severity'), severities], [self.tr('Node'), nodes], [self.tr('Topic'), topics], [self.tr('Selected Message(s)')]]])
        menutext.append([self.tr('Copy Selected')])
        menutext.append([self.tr('Browse Selected')])

        menu = QMenu()
        submenus = []
        subsubmenus = []
        for item in menutext:
            if len(item) > 1:
                submenus.append(QMenu(item[0], menu))
                for subitem in item[1]:
                    if len(subitem) > 1:
                        subsubmenus.append(QMenu(subitem[0], submenus[-1]))
                        for subsubitem in subitem[1]:
                            subsubmenus[-1].addAction(subsubitem)
                        submenus[-1].addMenu(subsubmenus[-1])
                    else:
                        submenus[-1].addAction(subitem[0])
                menu.addMenu(submenus[-1])
            else:
                menu.addAction(item[0])
        action = menu.exec_(event.globalPos())

        if action is None or action == 0:
            return
        elif action.text() == self.tr('Browse Selected'):
            self._show_browsers()
        elif action.text() == self.tr('Copy Selected'):
            rowlist = []
            for current in self.table_view.selectionModel().selectedIndexes():
                rowlist.append(self._proxymodel.mapToSource(current).row())
            copytext = self._datamodel.get_selected_text(rowlist)
            if copytext is not None:
                clipboard = QApplication.clipboard()
                clipboard.setText(copytext)
        elif action.text() == self.tr('Selected Message(s)'):
            if action.parentWidget().title() == self.tr('Highlight'):
                self._process_highlight_exclude_filter(action.text(), 'Message', False)
            elif action.parentWidget().title() == self.tr('Exclude'):
                self._process_highlight_exclude_filter(action.text(), 'Message', True)
            else:
                raise RuntimeError("Menu format corruption in ConsoleWidget._rightclick_menu()")
        else:
            # This processes the dynamic list entries (severity, node and topic)
            try:
                roottitle = action.parentWidget().parentWidget().title()
            except:
                raise RuntimeError("Menu format corruption in ConsoleWidget._rightclick_menu()")

            if roottitle == self.tr('Highlight'):
                self._process_highlight_exclude_filter(action.text(), action.parentWidget().title(), False)
            elif roottitle == self.tr('Exclude'):
                self._process_highlight_exclude_filter(action.text(), action.parentWidget().title(), True)
            else:
                raise RuntimeError("Unknown Root Action %s selected in ConsoleWidget._rightclick_menu()" % roottitle)
        self.update_status()

    def update_status(self):
        """
        Sets the message display label to the current value
        """
        if self._datamodel.rowCount() == self._proxymodel.rowCount():
            tip = self.tr(self.tr('Displaying %s Messages' % (self._datamodel.rowCount())))
        else:
            tip = self.tr(self.tr('Displaying %s of %s Messages' % (self._proxymodel.rowCount(), self._datamodel.rowCount())))
        self.messages_label.setText(tip)

    def cleanup_browsers_on_close(self):
        for browser in self._browsers:
            browser.close()

    def _show_browsers(self):
        rowlist = []
        for current in self.table_view.selectionModel().selectedIndexes():
            rowlist.append(self._proxymodel.mapToSource(current).row())
        browsetext = self._datamodel.get_selected_text(rowlist)
        if browsetext is not None:
            self._browsers.append(TextBrowseDialog(browsetext))
            self._browsers[-1].show()

    def _handle_load_clicked(self, checked):
        filename = QFileDialog.getOpenFileName(self, self.tr('Load from File'), '.', self.tr('rqt_console message file {.csv} (*.csv)'))
        if filename[0] != '':
            try:
                handle = open(filename[0])
            except IOError as e:
                qWarning(str(e))
                return
            self.pause_button.setChecked(True)
            self._handle_pause_clicked(True)
            self._datamodel.load_from_file(handle)
            handle.close()
            self.update_status()

    def _handle_save_clicked(self, checked):
        filename = QFileDialog.getSaveFileName(self, 'Save to File', '.', self.tr('rqt_console msg file {.csv} (*.csv)'))
        if filename[0] != '':
            filename = filename[0]
            if filename[-4:] != '.csv':
                filename += '.csv'
            try:
                handle = open(filename, 'w')
            except IOError as e:
                qWarning(str(e))
                return
            self._proxymodel.save_to_file(handle)
            handle.close()
            self.update_status()

    def _handle_pause_clicked(self, checked):
        self._datamodel._paused = checked
        if self._datamodel._paused:
            self.pause_button.setIcon(self._recordicon)
            self.pause_button.setText(self.tr('Resume'))
        else:
            self.pause_button.setIcon(self._pauseicon)
            self.pause_button.setText(self.tr('Pause'))

    def _handle_column_resize_clicked(self):
        self.table_view.resizeColumnsToContents()

    def _handle_custom_keypress(self, event, old_keyPressEvent=QTableView.keyPressEvent):
        """
        Handles the delete key.
        The delete key removes the tableview's selected rows from the datamodel
        """
        if event.key() == Qt.Key_Delete and len(self._datamodel.get_message_list()) > 0:
            delete = QMessageBox.Yes
            if len(self.table_view.selectionModel().selectedIndexes()) == 0:
                delete = QMessageBox.question(self, self.tr('Message'), self.tr("Are you sure you want to delete all messages?"), QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            if delete == QMessageBox.Yes and event.key() == Qt.Key_Delete and event.modifiers() == Qt.NoModifier:
                rowlist = []
                for current in self.table_view.selectionModel().selectedIndexes():
                    rowlist.append(self._proxymodel.mapToSource(current).row())
                rowlist = list(set(rowlist))
                if self._datamodel.remove_rows(rowlist):
                    self.update_status()
                    return event.accept()
        return old_keyPressEvent(self.table_view, event)

    def _handle_mouse_double_click(self, event, old_doubleclickevent=QTableView.mouseDoubleClickEvent):
        if event.buttons() & Qt.LeftButton and event.modifiers() == Qt.NoModifier:
            self._show_browsers()
            event.accept()
        return old_doubleclickevent(self.table_view, event)

    def _handle_mouse_press(self, event, old_pressEvent=QTableView.mousePressEvent):
        if event.buttons() & Qt.RightButton and event.modifiers() == Qt.NoModifier:
            self._rightclick_menu(event)
            event.accept()
        return old_pressEvent(self.table_view, event)

    def save_settings(self, plugin_settings, instance_settings):
        instance_settings.set_value('settings_exist', True)
        
        instance_settings.set_value('table_splitter', self.table_splitter.saveState())
        instance_settings.set_value('filter_splitter', self.filter_splitter.saveState())

        instance_settings.set_value('paused', self.pause_button.isChecked())
        instance_settings.set_value('show_highlighted_only', self.highlight_exclude_button.isChecked())

        exclude_filters = []
        for index, item in enumerate(self._exclude_filters):
            exclude_filters.append(item[2])
            filter_settings = instance_settings.get_settings('exclude_filter_' + str(index))
            item[1].save_settings(filter_settings)
        instance_settings.set_value('exclude_filters', exclude_filters)

        highlight_filters = []
        for index, item in enumerate(self._highlight_filters):
            highlight_filters.append(item[2])
            filter_settings = instance_settings.get_settings('highlight_filter_' + str(index))
            item[1].save_settings(filter_settings)
        instance_settings.set_value('highlight_filters', highlight_filters)

    def restore_settings(self, pluggin_settings, instance_settings):
        if instance_settings.contains('table_splitter'):
            self.table_splitter.restoreState(instance_settings.value('table_splitter'))
        else:
            self.table_splitter.setSizes([1000, 100])
        if instance_settings.contains('filter_splitter'):
            self.filter_splitter.restoreState(instance_settings.value('filter_splitter'))
        else:
            self.table_splitter.setSizes([500, 500])

        self.pause_button.setChecked(instance_settings.value('paused') in [True, 'true'])
        self._handle_pause_clicked(self.pause_button.isChecked())
        self.highlight_exclude_button.setChecked(instance_settings.value('show_highlighted_only') in [True, 'true'])
        self._proxymodel.set_show_highlighted_only(self.highlight_exclude_button.isChecked())

        for item in self._exclude_filters:
            item[1].delete_button.setChecked(True)
        self._delete_exclude_filter()
        if instance_settings.contains('exclude_filters'):
            exclude_filters = instance_settings.value('exclude_filters')
            if exclude_filters is not None:
                for index, item in enumerate(exclude_filters):
                    self._add_exclude_filter(item)
                    filter_settings = instance_settings.get_settings('exclude_filter_' + str(index))
                    self._exclude_filters[-1][1].restore_settings(filter_settings)
        else:
            self._add_exclude_filter('severity')

        for item in self._highlight_filters:
            item[1].delete_button.setChecked(True)
        self._delete_highlight_filter()
        if instance_settings.contains('highlight_filters'):
            highlight_filters = instance_settings.value('highlight_filters')
            if highlight_filters is not None:
                for index, item in enumerate(highlight_filters):
                    self._add_highlight_filter(item)
                    filter_settings = instance_settings.get_settings('highlight_filter_' + str(index))
                    self._highlight_filters[-1][1].restore_settings(filter_settings)
        else:
            self._add_highlight_filter('message')
