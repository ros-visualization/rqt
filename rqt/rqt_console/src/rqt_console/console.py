import os
import new
import roslib, rospy
roslib.load_manifest('rqt_console')

from qt_gui.plugin import Plugin
from qt_gui.qt_binding_helper import loadUi
from QtGui import QWidget, QDialog, QInputDialog, QTableView, QMessageBox, QHeaderView
from QtCore import qDebug, Qt, QTimer, Slot

from message_data_model import MessageDataModel
from custom_widgets import MainWindow, SetupDialog, TimeDialog

class Console(Plugin):
    def __init__(self, context):
        super(Console, self).__init__(context)
        # give QObjects reasonable names
        self.setObjectName('Console')

        # create QWidget
        self._mainwindow = MainWindow()
        # get path to UI file which is a sibling of this file
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'console.ui')
        # extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._mainwindow)
        # give QObjects reasonable names
        self._mainwindow.setObjectName('ConsoleUi')
        # add widget to the user interface
        context.add_widget(self._mainwindow)
        self._datamodel = MessageDataModel()
        self._mainwindow.table_view.setModel(self._datamodel)

        self._mainwindow.table_view.setVisible(False)
        self._columnwidth = (600, 140, 200, 360, 200, 600)
        for idx, width in enumerate(self._columnwidth):
            self._mainwindow.table_view.horizontalHeader().resizeSection(idx, width)

        self._mainwindow.table_view.setVisible(True)

        self._mainwindow.table_view.mouseDoubleClickEvent = self.custom_doubleclick
        self._mainwindow.table_view.keyPressEvent = self.custom_keypress
        self._mainwindow.keyPressEvent = self.custom_keypress


        self._setupdialog = SetupDialog(context, self.message_callback)
        self._timedialog = TimeDialog()


    def message_callback(self, data):
        if self._mainwindow.logging_checkbox.isChecked():
            self._datamodel.insertRows(data)
            self.reset_status()
            self._mainwindow.table_view.reset()
    
    def custom_doubleclick(self, event, old_clickEvent=QTableView.mouseDoubleClickEvent):

        columnclicked = self._mainwindow.table_view.columnAt(event.x())
        if columnclicked == 0:
            text, ok = QInputDialog.getText(QWidget(), 'Message filter', 'Enter text (leave blank for no filtering):')
        elif columnclicked == 1:
            text, ok = QInputDialog.getItem(QWidget(), 'Severity filter', 'Include only:', ['All', 'Debug', 'Info', 'Warning', 'Error', 'Fatal'], 0, False)
        elif columnclicked == 2:
            text, ok = QInputDialog.getItem(QWidget(), 'Node filter', 'Include only:', ['All'] + self._datamodel.get_unique_col_data(columnclicked), 0, False)
        elif columnclicked == 3:
            self._clear_filter = False
            def handle_ignore():
                self._clear_filter = True
            self._timedialog.ignore_button_clicked.connect(handle_ignore)
            
            indexes = self._mainwindow.table_view.selectionModel().selectedIndexes()
            if len(indexes) == 0:
                self._timedialog.set_time()
            else:
            #get the current selection get the min and max times from this range
            #and set them as the min/max
                rowlist = []
                for current in indexes:
                    rowlist.append(current.row())
                rowlist = list(set(rowlist))
                rowlist.sort()
                
                mintime = self._datamodel.get_data(rowlist[0],3)
                maxtime = self._datamodel.get_data(rowlist[-1],3)
                mintime = mintime[:mintime.find('.')]
                maxtime = maxtime[:maxtime.find('.')]
                self._timedialog.set_time(int(mintime),int(maxtime))

            ok = self._timedialog.exec_()
            self._timedialog.ignore_button_clicked.disconnect(handle_ignore)
            ok = (ok == 1)
            if self._clear_filter:
                text = ''
            else:
                text = str(self._timedialog.min_dateedit.dateTime().toTime_t()) + ':' + str(self._timedialog.max_dateedit.dateTime().toTime_t())
        elif columnclicked == 4:
            unique_list = set()
            for topiclists in self._datamodel.get_unique_col_data(columnclicked):
                for item in topiclists.split(','):
                    unique_list.add(item.strip())
            unique_list = list(unique_list)
            text, ok = QInputDialog.getItem(QWidget(), 'Topic filter', 'Include only:', ['All'] + unique_list , 0, False)
        elif columnclicked == 5:
            text, ok = QInputDialog.getText(QWidget(), 'Location Filter', 'Enter text (leave blank for no filtering:')
        else:
            ok = False
        if ok:
            if text == 'All':
                text = ''
            self._datamodel.alter_filter_text(columnclicked, text)
            self.reset_status()
            return event.accept()
        return old_clickEvent(self._mainwindow.table_view, event)

    def custom_keypress(self, event, old_keyPressEvent=QTableView.keyPressEvent):
        if event.key() == Qt.Key_Delete:
            delete = QMessageBox.Yes
            if len(self._mainwindow.table_view.selectionModel().selectedIndexes()) == 0:
                delete = QMessageBox.question(self._mainwindow, 'Message', "Are you sure you want to delete all messages?", QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            if delete == QMessageBox.Yes and event.key() == Qt.Key_Delete and event.modifiers() == Qt.NoModifier:
                if self._datamodel.remove_rows(self._mainwindow.table_view.selectionModel().selectedIndexes()):
                    self.reset_status()
                    return event.accept()
        return old_keyPressEvent(self._mainwindow.table_view, event)

    def shutdown_plugin(self):
        self._setupdialog.unsub_topic()
        self._setupdialog.close()

    def save_settings(self, plugin_settings, instance_settings):
        for index, member in enumerate(self._datamodel.message_members()):
            instance_settings.set_value(member,self._datamodel.get_filter(index))

    def restore_settings(self, plugin_settings, instance_settings):
        for index, member in enumerate(self._datamodel.message_members()):
            self._datamodel.set_filter(index, instance_settings.value(member))

    def trigger_configuration(self):
        self._setupdialog.refresh_nodes()
        self._setupdialog.show()
        self._setupdialog.node_list.item(0).setSelected(True)
        self._setupdialog.node_changed(0)

    def reset_status(self):
        if self._datamodel.count() == self._datamodel.count(True):
            tip = self._mainwindow.tr('Displaying %s Messages' % (self._datamodel.count())) 
        else:
            tip = self._mainwindow.tr('Displaying %s of %s Messages' % (self._datamodel.count(True),self._datamodel.count())) 
        self._mainwindow.setStatusTip(tip)

