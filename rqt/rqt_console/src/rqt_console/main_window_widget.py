import os

from qt_gui.qt_binding_helper import loadUi
from QtGui import QFileDialog, QIcon, QInputDialog, QLineEdit, QMenu, QMessageBox, QTableView, QWidget
from QtCore import Qt

from custom_widgets import ListDialog, TimeDialog 

class MainWindow(QWidget):
    def __init__(self, proxymodel):
        super(MainWindow, self).__init__()
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'console.ui')
        loadUi(ui_file, self)
        self.setObjectName('ConsoleUi')
        self.table_view.setModel(proxymodel)
        self._proxymodel = proxymodel
        self._datamodel = proxymodel.sourceModel()

        self._columnwidth = (600, 140, 200, 430, 200, 600)
        for idx, width in enumerate(self._columnwidth):
            self.table_view.horizontalHeader().resizeSection(idx, width)
        self.table_view.sortByColumn(3,Qt.DescendingOrder)

        self.pause_button.setIcon(QIcon.fromTheme('media-playback-pause'))
        self.open_button.setIcon(QIcon.fromTheme('document-open'))
        self.save_button.setIcon(QIcon.fromTheme('document-save'))

        self.open_button.clicked[bool].connect(self.open_press_handler)
        self.save_button.clicked[bool].connect(self.save_press_handler)
        self.table_view.mousePressEvent = self.mouse_press_handler
        self.table_view.keyPressEvent = self.custom_keypress_handler

        self._paused = False
        self.pause_button.clicked[bool].connect(self.pause_press_handler)

    def open_press_handler(self, b):
        filename = QFileDialog.getOpenFileName(self, 'Load from File', '.')
        if filename[0] != '':
            fileHandle = open(filename[0])
            self._datamodel.open_from_file(fileHandle)
            fileHandle.close()
            self.update_status()
    
    def save_press_handler(self, b):
        filename = QFileDialog.getSaveFileName(self, 'Save to File', '.')
        if filename[0] != '':
            fileHandle = open(filename[0], 'w')
            self._datamodel.save_to_file(fileHandle)
            fileHandle.close()
            self.update_status()

    def pause_press_handler(self, b):
        self._paused = not self._paused
        if self._paused:
            self.pause_button.setIcon(QIcon.fromTheme('media-record'))
            self.pause_button.setText('Resume')
        else:
            self.pause_button.setIcon(QIcon.fromTheme('media-playback-pause'))
            self.pause_button.setText('Pause')

    def custom_keypress_handler(self, event, old_keyPressEvent=QTableView.keyPressEvent):
        if event.key() == Qt.Key_Delete and len(self._datamodel.get_message_list()) > 0:
            delete = QMessageBox.Yes
            if len(self.table_view.selectionModel().selectedIndexes()) == 0:
                delete = QMessageBox.question(self, 'Message', "Are you sure you want to delete all messages?", QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            if delete == QMessageBox.Yes and event.key() == Qt.Key_Delete and event.modifiers() == Qt.NoModifier:
                rowlist = []
                for current in self.table_view.selectionModel().selectedIndexes():
                    rowlist.append(self._proxymodel.mapToSource(current).row())
                rowlist = list(set(rowlist))
                if self._datamodel.remove_rows(rowlist):
                    self.update_status()
                    return event.accept()
        return old_keyPressEvent(self.table_view, event)
    
    def mouse_press_handler(self, 
                            event,
                            old_pressEvent=QTableView.mousePressEvent):
        if event.buttons() & Qt.RightButton and event.modifiers() == Qt.NoModifier:
            self.rightclick_menu(event)
            return event.accept()
        return old_pressEvent(self.table_view, event)

    def show_filter_input_dialog(self, pos):
        col = self.table_view.columnAt(pos.x())
        if col == 0:
            text, ok = QInputDialog.getText(QWidget(), 'Message filter', 'Enter text (leave blank for no filtering):', QLineEdit.Normal, self._proxymodel.get_filter(col))
        elif col == 1:
            textlist, ok = ListDialog.show('Severity filter', 'Include only:', ['Debug', 'Info', 'Warning', 'Error', 'Fatal'], self._proxymodel.get_filter(col))
            text = ''
            for item in textlist:
                text += item + self._proxymodel.get_or()
            text = text[:-1]
        elif col == 2:
            textlist, ok = ListDialog.show('Node filter', 'Include only:', self._datamodel.get_unique_col_data(col), self._proxymodel.get_filter(col))
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
            textlist, ok = ListDialog.show('Topic filter', 'Include only:', unique_list, self._proxymodel.get_filter(col))
            text = ''
            for item in textlist:
                text += item + self._proxymodel.get_or()
            text = text[:-1]
        elif col == 5:
            text, ok = QInputDialog.getText(QWidget(), 'Location Filter', 'Enter text (leave blank for no filtering:', QLineEdit.Normal, self._proxymodel.get_filter(col))
        else:
            ok = False
        if ok:
            if text == 'All':
                text = ''
            self._proxymodel.set_filter(col, text)
            self.update_status()

    def process_inc_exc(self, col, exclude=False):
        prevfilter = self._proxymodel.get_filter(col)
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
 
    def rightclick_menu(self, event):
        # menutext[0] entries are added as Actions if menutext[1] does not exist
        # otherwise they are added as QMenus and  menutext[1] entries are added as subActions 
        menutext = []
        menutext.append(['Edit Filter'])
        if len(self.table_view.selectionModel().selectedIndexes()) != 0:
            menutext.append(['Exclude',['Node(s)','Message(s)']])
            menutext.append(['Include',['Node(s)','Message(s)']])
        menutext.append(['Clear Filter'])
        menutext.append(['Copy'])
        
        actions = []
        menu = QMenu()
        submenus = []
        submenuindex = -1
        for index, item in enumerate(menutext):
            if len(item) == 1:
                actions.append((item[0], menu.addAction(item[0])))
            else:
                submenus.append(QMenu())
                for subitem in item[1]:
                    actions.append((item[0] + '>' + subitem, submenus[-1].addAction(subitem)))
                submenus[-1].setTitle(item[0])
                menu.addMenu(submenus[-1])
                                
        actions = dict(actions)
        action = menu.exec_(event.globalPos())

        #actions are accessed by dict index menutext>submenutext
        col = self.table_view.columnAt(event.pos().x())
        if action is None or action == 0:
            return 
        elif action == actions['Clear Filter']:
            self._proxymodel.set_filter(col,'')
        elif action == actions['Edit Filter']:
            self.show_filter_input_dialog(event.pos())
        elif action == actions['Copy']:
            rowlist = []
            for current in self.table_view.selectionModel().selectedIndexes():
                rowlist.append(self._proxymodel.mapToSource(current).row())
            copytext = self._datamodel.get_selected_text(rowlist)
            if copytext is not None:
                clipboard = QApplication.clipboard()
                clipboard.setText(copytext)
        elif action == actions['Include>Node(s)']:
            self.process_inc_exc(2)
        elif action == actions['Include>Message(s)']:
            self.process_inc_exc(0)
        elif action == actions['Exclude>Node(s)']:
            self.process_inc_exc(2,True)
        elif action == actions['Exclude>Message(s)']:
            self.process_inc_exc(0,True)
        else:
            raise
        self.update_status()

    def is_paused(self):
        return self._paused

    def update_status(self):
        if self._datamodel.rowCount() == self._proxymodel.rowCount():
            tip = self.tr('Displaying %s Messages' % (self._datamodel.rowCount())) 
        else:
            tip = self.tr('Displaying %s of %s Messages' % (self._proxymodel.rowCount(),self._datamodel.rowCount())) 
        self.messages_label.setText(tip)

