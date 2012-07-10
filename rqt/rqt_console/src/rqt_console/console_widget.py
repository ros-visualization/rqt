import os

from qt_gui.qt_binding_helper import loadUi
from QtGui import QApplication, QFileDialog, QIcon, QInputDialog, QLineEdit, QMenu, QMessageBox, QTableView, QWidget
from QtCore import Qt, qWarning

from list_dialog import ListDialog
from time_dialog import TimeDialog 

class ConsoleWidget(QWidget):
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
        self.table_view.sortByColumn(3,Qt.DescendingOrder)
        
        self._pauseicon = QIcon.fromTheme('media-playback-pause')
        self._recordicon = QIcon.fromTheme('media-record')
        self.pause_button.setIcon(self._pauseicon)
        self.load_button.setIcon(QIcon.fromTheme('document-open'))
        self.save_button.setIcon(QIcon.fromTheme('document-save'))

        self.pause_button.clicked[bool].connect(self.pause_clicked_handler)
        self.load_button.clicked[bool].connect(self.load_clicked_handler)
        self.save_button.clicked[bool].connect(self.save_clicked_handler)
        self.table_view.mousePressEvent = self.mouse_press_handler
        self.table_view.keyPressEvent = self.custom_keypress_handler

    def load_clicked_handler(self, checked):
        filename = QFileDialog.getOpenFileName(self, self.tr('Load from File'), '.', self.tr('rqt_console message file ".csv" (*.csv)'))
        if filename[0] != '':
            try:
                handle = open(filename[0])
            except IOError, e:
                qWarning(str(e))
                return
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
    
    def mouse_press_handler(self, event, old_pressEvent=QTableView.mousePressEvent):
        if event.buttons() & Qt.RightButton and event.modifiers() == Qt.NoModifier:
            self._rightclick_menu(event)
            return event.accept()
        return old_pressEvent(self.table_view, event)

    def _show_filter_input_dialog(self, col):
        if col == 0:
            text, ok = QInputDialog.getText(QWidget(), self.tr('Message filter'), self.tr('Enter text (leave blank for no filtering):'), QLineEdit.Normal, self._proxymodel.get_filter(col))
        elif col == 1:
            severitylist = [self.tr('Debug'), self.tr('Info'), self.tr('Warning'), self.tr('Error'), self.tr('Fatal')]
            textlist, ok = ListDialog.show(self.tr('Severity filter'), self.tr('Include only:'), severitylist, self._proxymodel.get_filter(col))
            text = ''
            for item in textlist:
                text += item + self._proxymodel.get_or()
            text = text[:-1]
        elif col == 2:
            textlist, ok = ListDialog.show(self.tr('Node filter'), self.tr('Include only:'), self._datamodel.get_unique_col_data(col), self._proxymodel.get_filter(col))
            text = ''
            for item in textlist:
                text += item + self._proxymodel.get_or()
            text = text[:-1]
        elif col == 3:
            self._clear_filter = False
            def handle_ignore():
                self._clear_filter = True
            self._timedialog = TimeDialog()
            self._timedialog.ignore_button_clicked.connect(handle_ignore)
            
            indexes = self.table_view.selectionModel().selectedIndexes()
            if self._proxymodel.get_filter(col) != '':
                filter_ = self._proxymodel.get_filter(col)
                mintime, maxtime = filter_.split(':')
                self._timedialog.set_time(mintime,maxtime)
            elif len(indexes) != 0:
                rowlist = []
                for current in indexes:
                    rowlist.append(self._proxymodel.mapToSource(current).row())
                rowlist = list(set(rowlist))
                rowlist.sort()
                
                mintime, maxtime = self._datamodel.get_time_range(rowlist)
                self._timedialog.set_time(mintime,maxtime)
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
            textlist, ok = ListDialog.show(self.tr('Topic filter'), self.tr('Include only:'), unique_list, self._proxymodel.get_filter(col))
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
            self._proxymodel.set_filter(col, text)
            self.update_status()

    def _process_include_exclude_filter(self, selection, selectiontype, exclude=False):
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
            self._proxymodel.set_filter(col,newfilter)
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
            if prevfilter.find(nodetext) == -1:
                self._proxymodel.set_filter(col,newfilter)
 
    def _rightclick_menu(self, event):
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

        menutext = []
        menutext.append([self.tr('Exclude'),[[self.tr('Severity'), severities], [self.tr('Node'), nodes], [self.tr('Selected Message(s)')]]])
        menutext.append([self.tr('Include'),[[self.tr('Severity'), severities], [self.tr('Node'), nodes], [self.tr('Topic'), topics], [self.tr('Selected Message(s)')]]])
        menutext.append([self.tr('Clear Filter'), columns])
        menutext.append([self.tr('Edit Filter'), columns])
        menutext.append([self.tr('Copy Selected')])
        
        menu = QMenu()
        submenus = []
        subsubmenus = []
        for item in menutext:
            if len(item) > 1:
                submenus.append(QMenu(item[0],menu))
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
        elif [action.text()] in columns:
            if action.parentWidget().title() == self.tr('Edit Filter'):
                for index, col in enumerate(columns):
                    if action.text() == col[0]:
                        self._show_filter_input_dialog(index)
            else:
                for index, col in enumerate(columns):
                    if action.text() == col[0]:
                        self._proxymodel.set_filter(index,'')
        else:
            try:
                roottitle = action.parentWidget().parentWidget().title()
            except:
                raise RuntimeError(self.tr("Menu format corruption in ConsoleWidget._rightclick_menu()"))
                return

            if roottitle == self.tr('Include'):
                self._process_include_exclude_filter(action.text(),action.parentWidget().title(), False)
            elif roottitle == self.tr('Exclude'):
                self._process_include_exclude_filter(action.text(),action.parentWidget().title(), True)
            else:
                raise RuntimeError(self.tr("Unknown Root Action %s selected in ConsoleWidget._rightclick_menu()" % roottitle))
                return
        self.update_status()

    def update_status(self):
        if self._datamodel.rowCount() == self._proxymodel.rowCount():
            tip = self.tr(self.tr('Displaying %s Messages' % (self._datamodel.rowCount())))
        else:
            tip = self.tr(self.tr('Displaying %s of %s Messages' % (self._proxymodel.rowCount(),self._datamodel.rowCount())))
        self.messages_label.setText(tip)

