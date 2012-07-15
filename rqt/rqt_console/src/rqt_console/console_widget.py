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

from qt_gui.qt_binding_helper import loadUi
from QtGui import QApplication, QCursor, QFileDialog, QIcon, QInputDialog, QLineEdit, QMenu, QMessageBox, QTableView, QTableWidgetItem, QWidget
from QtCore import Qt, qWarning

#from list_dialog import ListDialog
#from time_dialog import TimeDialog
from text_browse_dialog import TextBrowseDialog

# For Filter Factory
from filter_wrapper_widget import FilterWrapperWidget
from severity_filter_widget import SeverityFilterWidget
from list_filter_widget import ListFilterWidget
from time_filter_widget import TimeFilterWidget
from text_filter_widget import TextFilterWidget
from severity_filter import SeverityFilter
from topic_filter import TopicFilter
from node_filter import NodeFilter
from time_filter import TimeFilter
from message_filter import MessageFilter
from location_filter import LocationFilter

# TODO construct and list more filters


class ConsoleWidget(QWidget):
    """
    Primary widget for the rqt_console plugin.
    """
    def __init__(self, proxymodel):
        super(ConsoleWidget, self).__init__()
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'console.ui')
        loadUi(ui_file, self)
        self.setObjectName('ConsoleWidget')
        self.table_view.setModel(proxymodel)
        self._proxymodel = proxymodel
        self._datamodel = proxymodel.sourceModel()

        self._columnwidth = (600, 140, 200, 430, 200, 600)
        for idx, width in enumerate(self._columnwidth):
            self.table_view.horizontalHeader().resizeSection(idx, width)
        self.table_view.sortByColumn(3, Qt.DescendingOrder)
        
        self._pauseicon = QIcon.fromTheme('media-playback-pause')
        self._recordicon = QIcon.fromTheme('media-record')
        self.pause_button.setIcon(self._pauseicon)
        self.load_button.setIcon(QIcon.fromTheme('document-open'))
        self.save_button.setIcon(QIcon.fromTheme('document-save'))

        self.pause_button.clicked[bool].connect(self.pause_clicked_handler)
        self.load_button.clicked[bool].connect(self.load_clicked_handler)
        self.save_button.clicked[bool].connect(self.save_clicked_handler)

        self.table_view.mouseDoubleClickEvent = self.mouse_double_click_handler
        self.table_view.mousePressEvent = self.mouse_press_handler
        self.table_view.keyPressEvent = self.custom_keypress_handler
        self.severitylist = [self.tr('Debug'), self.tr('Info'), self.tr('Warning'), self.tr('Error'), self.tr('Fatal')]
        
        # These are lists of Tuples = (,)
        self._exclude_filters = []
        self._highlight_filters = []
        
        self.highlight_group_box.clicked[bool].connect(self._proxymodel.set_show_highlighted_only)
        self.highlight_group_box.toggled[bool].connect(self.handle_reenable_highlight_filters)
        self.exclude_group_box.toggled[bool].connect(self.handle_reenable_exclude_filters)
        
        self.add_highlight_button.clicked.connect(self.add_highlight_filter)
        self.add_exclude_button.clicked.connect(self.add_exclude_filter)
        self.filter_factory = {0: ('Message Filter', MessageFilter, TextFilterWidget, []),
                               1: ('Severity Filter', SeverityFilter, SeverityFilterWidget, self.severitylist),
                               2: ('Node Filter', NodeFilter, ListFilterWidget, [self._datamodel.get_unique_col_data, 2 ]),
                               3: ('Time Filter', TimeFilter, TimeFilterWidget, [self.get_time_range_from_selection]),
                               4: ('Topic Filter', TopicFilter, ListFilterWidget, [self._datamodel.get_unique_col_data, 4 ]),
                               5: ('Location Filter', MessageFilter, TextFilterWidget, [])}

#TODO these calls aren't going to work need to find a way to make it work

        # list of TextBrowserDialogs to close when cleaning up
        self._browsers = []

        # This defaults the filters panel to closed
        self.table_splitter.setSizes([1, 0])
        self.exclude_table.resizeColumnsToContents()
        self.highlight_table.resizeColumnsToContents()

    def get_time_range_from_selection(self):
        rowlist = []
        indexes = self.table_view.selectionModel().selectedIndexes()
        
        if len(indexes) != 0:
            for current in indexes:
                rowlist.append(self._proxymodel.mapToSource(current).row())
            rowlist = list(set(rowlist))
            rowlist.sort()
            
            mintime, maxtime = self._datamodel.get_time_range(rowlist)
            return (mintime, maxtime)
        return (-1,-1)

    def handle_reenable_exclude_filters(self, enabled):
        # TODO fix this function it does not reenable the groupbox components
        self.highlight_group_box.setEnabled(True)
        for item in self._exclude_filters:
            item[1].setEnabled(True)
            item[1].enable_all_children()

    def handle_reenable_highlight_filters(self, enabled):
        # TODO fix this function it does not reenable the groupbox components
        self.highlight_group_box.setEnabled(True)
        for item in self._highlight_filters:
            item[1].setEnabled(True)
            item[1].enable_all_children()

# TODO combine highlight/exclude duplicated code functions
    def delete_highlight_filter(self):
        for index, item in enumerate(self._highlight_filters):
            if item[1].delete_button.isChecked():
                if self._proxymodel.delete_highlight_filter(index):
                    self.highlight_table.removeCellWidget(index, 0)
                    self.highlight_table.removeRow(index)
                    item[0].filter_changed_signal.disconnect(self._proxymodel.filters_changed_handler)
                    item[1].delete_button.clicked.disconnect(self.delete_highlight_filter)
                    del self._highlight_filters[index]

#TODO delete checked filters
    def delete_exclude_filter(self):
        for index, item in enumerate(self._exclude_filters):
            if item[1].delete_button.isChecked():
                if self._proxymodel.delete_exclude_filter(index):
                    self.exclude_table.removeCellWidget(index, 0)
                    self.exclude_table.removeRow(index)
                    item[0].filter_changed_signal.disconnect(self._proxymodel.filters_changed_handler)
                    item[1].delete_button.clicked.disconnect(self.delete_exclude_filter)
                    del self._exclude_filters[index]

    def add_highlight_filter(self, filter_index=None):
        """
        Adds an exclude filter (needs to be generalized)
        """
        if filter_index is False:
            filter_index = -1
            filter_select_menu = QMenu()
            for index in range(len(self.filter_factory)):
                # flattens the _highlight filters list and only adds the item if it doesn't already exist
                if not self.filter_factory[index][1] in [type(item) for sublist in self._highlight_filters for item in sublist]:
                    filter_select_menu.addAction(self.filter_factory[index][0])
            action = filter_select_menu.exec_(QCursor.pos()) #  TODO get a handle on the mouse position, pass it in 
            if action is None:
                return
            for index in range(len(self.filter_factory)):
                if self.filter_factory[index][0] == action.text():
                    filter_index = index
            if filter_index == -1:   
                return

        newfilter = self.filter_factory[filter_index][1]()
        newwidget = self.filter_factory[filter_index][2](newfilter, self.filter_factory[filter_index][3])

        index = len(self._highlight_filters)
        self._highlight_filters.append((newfilter, FilterWrapperWidget(newwidget)))
        self._proxymodel.add_highlight_filter(newfilter)
        newfilter.filter_changed_signal.connect(self._proxymodel.filters_changed_handler)
        self._highlight_filters[index][1].delete_button.clicked.connect(self.delete_highlight_filter)
        self._datamodel.rowsInserted.connect(self._highlight_filters[index][1].repopulate)

        self.highlight_table.insertRow(index)
        self.highlight_table.setCellWidget(index, 0, self._highlight_filters[index][1])
        self.highlight_table.resizeColumnsToContents()
        self.highlight_table.resizeRowsToContents()
        newfilter.filter_changed_signal.emit()

    def add_exclude_filter(self, filter_index=False):
        """
        Adds an exclude filter (needs to be generalized)
        """
        if filter_index is False:
            filter_index = -1
            filter_select_menu = QMenu()
            for index in range(len(self.filter_factory)):
                # flattens the _exclude filters list and only adds the item if it doesn't already exist
                if not self.filter_factory[index][1] in [type(item) for sublist in self._exclude_filters for item in sublist]:
                    filter_select_menu.addAction(self.filter_factory[index][0])
            action = filter_select_menu.exec_(QCursor.pos()) #  TODO get a handle on the mouse position, pass it in 
            if action is None:
                return
            for index in range(len(self.filter_factory)):
                if self.filter_factory[index][0] == action.text():
                    filter_index = index
            if filter_index == -1:   
                return

        newfilter = self.filter_factory[filter_index][1]()
        newwidget = self.filter_factory[filter_index][2](newfilter, self.filter_factory[filter_index][3])

        index = len(self._exclude_filters)
        self._exclude_filters.append((newfilter, FilterWrapperWidget(newwidget)))
        self._proxymodel.add_exclude_filter(newfilter)
        newfilter.filter_changed_signal.connect(self._proxymodel.filters_changed_handler)
        self._exclude_filters[index][1].delete_button.clicked.connect(self.delete_exclude_filter)
        self._datamodel.rowsInserted.connect(self._exclude_filters[index][1].repopulate)
        
        self.exclude_table.insertRow(index)
        self.exclude_table.setCellWidget(index, 0, self._exclude_filters[index][1])
        self.exclude_table.resizeColumnsToContents()
        self.exclude_table.resizeRowsToContents()
        newfilter.filter_changed_signal.emit()

    def cleanup_on_close(self):
        for browser in self._browsers:
            browser.close()

    def load_clicked_handler(self, checked):
        filename = QFileDialog.getOpenFileName(self, self.tr('Load from File'), '.', self.tr('rqt_console message file ".csv" (*.csv)'))
        if filename[0] != '':
            try:
                handle = open(filename[0])
            except IOError, e:
                qWarning(str(e))
                return
            self.pause_button.setChecked(True)
            self.pause_clicked_handler(True)
            self._datamodel.load_from_file(handle)
            handle.close()
            self.update_status()
    
    def save_clicked_handler(self, checked):
        filename = QFileDialog.getSaveFileName(self, 'Save to File', '.', self.tr('rqt_console msg file ".csv" (*.csv)'))
        if filename[0] != '':
            try:
                handle = open(filename[0], 'w')
            except IOError, e:
                qWarning(str(e))
                return
            self._datamodel.save_to_file(handle)
            handle.close()
            self.update_status()

    def pause_clicked_handler(self, checked):
        self._datamodel._paused = checked
        if self._datamodel._paused:
            self.pause_button.setIcon(self._recordicon)
            self.pause_button.setText(self.tr('Resume'))
        else:
            self.pause_button.setIcon(self._pauseicon)
            self.pause_button.setText(self.tr('Pause'))

    def custom_keypress_handler(self, event, old_keyPressEvent=QTableView.keyPressEvent):
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
    
    def mouse_double_click_handler(self, event, old_doubleclickevent=QTableView.mouseDoubleClickEvent):
        if event.buttons() & Qt.LeftButton and event.modifiers() == Qt.NoModifier:
            self.show_browsers()
            event.accept()
        return old_doubleclickevent(self.table_view, event)

    def mouse_press_handler(self, event, old_pressEvent=QTableView.mousePressEvent):
        if event.buttons() & Qt.RightButton and event.modifiers() == Qt.NoModifier:
            self._rightclick_menu(event)
            event.accept()
        return old_pressEvent(self.table_view, event)

    def _show_filter_input_dialog(self, col):
        """
        Displays the correct filtering dialog based on the col variable.
        """
        if col == 0:
            text, ok = QInputDialog.getText(QWidget(), self.tr('Message filter'), self.tr('Enter text (leave blank for no filtering):'), QLineEdit.Normal, self._proxymodel.get_filter(col))
        elif col == 1:
#            textlist, ok = ListDialog.show(self.tr('Severity filter'), self.tr('Include only:'), self.severitylist, self._proxymodel.get_filter(col))
            text = ''
            for item in textlist:
                text += item + self._proxymodel.get_or()
            text = text[:-1]
        elif col == 2:
#            textlist, ok = ListDialog.show(self.tr('Node filter'), self.tr('Include only:'), self._datamodel.get_unique_col_data(col), self._proxymodel.get_filter(col))
            text = ''
            for item in textlist:
                text += item + self._proxymodel.get_or()
            text = text[:-1]
        elif col == 3:
            self._clear_filter = False
            def handle_ignore():
                self._clear_filter = True
#            self._timedialog = TimeDialog()
            self._timedialog.ignore_button_clicked.connect(handle_ignore)
            
            indexes = self.table_view.selectionModel().selectedIndexes()
            if self._proxymodel.get_filter(col) != '':
                filter_ = self._proxymodel.get_filter(col)
                mintime, maxtime = filter_.split(':')
                self._timedialog.set_time(mintime, maxtime)
            elif len(indexes) != 0:
                rowlist = []
                for current in indexes:
                    rowlist.append(self._proxymodel.mapToSource(current).row())
                rowlist = list(set(rowlist))
                rowlist.sort()
                
                mintime, maxtime = self._datamodel.get_time_range(rowlist)
                self._timedialog.set_time(mintime, maxtime)
            else:
                self._timedialog.set_time()
            ok = self._timedialog.exec_()
            self._timedialog.ignore_button_clicked.disconnect(handle_ignore)
            ok = (ok == 1)
            if self._clear_filter:
                text = ''
            else:
                mintime = str(self._timedialog.min_dateedit.dateTime().toTime_t()) + self._timedialog.min_dateedit.dateTime().toString('.zzz')
                maxtime = str(self._timedialog.max_dateedit.dateTime().toTime_t()) + self._timedialog.max_dateedit.dateTime().toString('.zzz')
                text = mintime + ':' + maxtime
        elif col == 4:
            unique_list = set()
            for topiclists in self._datamodel.get_unique_col_data(col):
                for item in topiclists.split(','):
                    unique_list.add(item.strip())
            unique_list = list(unique_list)
#            textlist, ok = ListDialog.show(self.tr('Topic filter'), self.tr('Include only:'), unique_list, self._proxymodel.get_filter(col))
            text = ''
            for item in textlist:
                text += item + self._proxymodel.get_or()
            text = text[:-1]
        elif col == 5:
            text, ok = QInputDialog.getText(QWidget(), self.tr('Location Filter'), self.tr('Enter text (leave blank for no filtering):'), QLineEdit.Normal, self._proxymodel.get_filter(col))
        else:
            ok = False
        if ok:
            if text == 'All':
                text = ''
#            self._proxymodel.set_filter(col, text)
            self.update_status()

    def _process_include_exclude_filter(self, selection, selectiontype, exclude=False):
        """
        Modifies the relevant filters (based on selectiontype) to remove (exclude=True) 
        or include (exclude=False) the selection from the dataset in the tableview 
        """
        types = {self.tr('Node'):2, self.tr('Topic'):4, self.tr('Severity'):1, self.tr('Message'):0}
        try:
            col = types[selectiontype]
        except:
            raise RuntimeError(self.tr("Bad Column name in ConsoleWidget._process_include_exclude_filter()"))
            return
        prevfilter = self._proxymodel.get_filter(col)
        if col != 0:
            if prevfilter == '':
                if exclude:
                    newlist = self._datamodel.get_unique_col_data(col)
                    temp = []
                    for item in newlist:
                        if item.find(', ') == -1:
                            temp.append(item)
                        else:
                            temp = temp + item.split(', ')
                    newlist = list(set(temp))
                    del newlist[newlist.index(selection)]
                    newfilter = '|'.join(newlist)
                else:
                    newfilter = selection
            else:
                if exclude:
                    newlist = prevfilter.split('|')
                    if selection in newlist:
                        del newlist[newlist.index(selection)]
                    newfilter = '|'.join(newlist)
                    if newfilter == '':
                        qWarning(self.tr('You can not exclude the only remaining entries in the list.'))
                        return
                else: 
                    newlist = prevfilter.split('|')
                    if not selection in newlist:
                        newlist.append(selection)
                    newfilter = '|'.join(newlist)
#            self._proxymodel.set_filter(col, newfilter)
            return
        else:
            if prevfilter != '':
                prevfilter = '(' + prevfilter + ')' + self._proxymodel.get_and()
            num_selected = len(self.table_view.selectionModel().selectedIndexes())/6
            nodetext = ''
            for index in range(num_selected):
                addtext = self.table_view.selectionModel().selectedIndexes()[num_selected*col+index].data()
                if exclude:
                    addtext = self._proxymodel.get_not() + addtext
                nodetext += addtext
                if exclude:
                    nodetext += self._proxymodel.get_and()
                else:
                    nodetext += self._proxymodel.get_or()
            nodetext = nodetext[:-1]
            newfilter = prevfilter + nodetext
#            if prevfilter.find(nodetext) == -1:
#                self._proxymodel.set_filter(col, newfilter)
 
    def _rightclick_menu(self, event):
        """
        Dynamically builds the rightclick menu based on the unique column data
        from the passed in datamodel and then launches it modally
        """
        #TODO FIX THIS FUNCTION FOR USE WITH NEW FILTERS
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
# TODO make this functionality work again
#        menutext.append([self.tr('Exclude'), [[self.tr('Severity'), severities], [self.tr('Node'), nodes], [self.tr('Selected Message(s)')]]])
#        menutext.append([self.tr('Include'), [[self.tr('Severity'), severities], [self.tr('Node'), nodes], [self.tr('Topic'), topics], [self.tr('Selected Message(s)')]]])
#        menutext.append([self.tr('Clear Filter'), columns])
#        menutext.append([self.tr('Edit Filter'), columns])
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
            self.show_browsers()
        elif action.text() == self.tr('Copy Selected'):
            rowlist = []
            for current in self.table_view.selectionModel().selectedIndexes():
                rowlist.append(self._proxymodel.mapToSource(current).row())
            copytext = self._datamodel.get_selected_text(rowlist)
            if copytext is not None:
                clipboard = QApplication.clipboard()
                clipboard.setText(copytext)
        elif action.text() == self.tr('Selected Message(s)'):
            if action.parentWidget().title() == self.tr('Include'):
                self._process_include_exclude_filter(action.text(), 'Message', False)
            elif action.parentWidget().title() == self.tr('Exclude'):
                self._process_include_exclude_filter(action.text(), 'Message', True)
            else:
                raise RuntimeError(self.tr("Menu format corruption in ConsoleWidget._rightclick_menu()"))
                return
#        elif [action.text()] in columns:
#            # This processes the clear and edit filter menu items
#            if action.parentWidget().title() == self.tr('Edit Filter'):
#                for index, col in enumerate(columns):
#                    if action.text() == col[0]:
#                        self._show_filter_input_dialog(index)
#            elif action.parentWidget().title() == self.tr('Edit Filter'):
#                for index, col in enumerate(columns):
#                    if action.text() == col[0]:
#                        self._proxymodel.set_filter(index, '')
#            else:
#                raise RuntimeError(self.tr("Menu format corruption in ConsoleWidget._rightclick_menu()"))
#                return
        else:
            # This processes the dynamic list entries (severity, node and topic)
            try:
                roottitle = action.parentWidget().parentWidget().title()
            except:
                raise RuntimeError(self.tr("Menu format corruption in ConsoleWidget._rightclick_menu()"))
                return

            if roottitle == self.tr('Include'):
                self._process_include_exclude_filter(action.text(), action.parentWidget().title(), False)
            elif roottitle == self.tr('Exclude'):
                self._process_include_exclude_filter(action.text(), action.parentWidget().title(), True)
            else:
                raise RuntimeError(self.tr("Unknown Root Action %s selected in ConsoleWidget._rightclick_menu()" % roottitle))
                return
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

    def show_browsers(self):
        rowlist = []
        for current in self.table_view.selectionModel().selectedIndexes():
            rowlist.append(self._proxymodel.mapToSource(current).row())
        browsetext = self._datamodel.get_selected_text(rowlist)
        if browsetext is not None:
            self._browsers.append(TextBrowseDialog(browsetext))
        self._browsers[-1].show()

