import time
from filtered_list import FilteredList

from QtCore import QAbstractTableModel, QByteArray, QMimeData, QModelIndex, Qt, QDateTime, pyqtSignal, Signal
from QtGui import QWidget

class MessageDataModel(QAbstractTableModel):

    def __init__(self, editable=False):
        QAbstractTableModel.__init__(self)
        self._messages = FilteredList()
        self._editable = False

        self._severity = {1: 'Debug', 2: 'Info', 4:'Warn', 8:'Error', 16: 'Fatal'}

    def rowCount(self, parent=None):
        return len(self._messages.get_message_list())

    def columnCount(self, parent=None):
        return self._messages.columnCount()

    def data(self, index, role=None):
        #return the index's data 
        if role is None:
            role = Qt.DisplayRole
        messagelist = self._messages.get_message_list()
        if index.row() >= 0 and index.row() < len(messagelist):
            if index.column() < 0 or index.column() >= messagelist[index.row()].CountElements():
                raise IndexError
            if role == Qt.DisplayRole:
                elements = self._messages.message_members()
                if elements[index.column()] == '_time':
                    time = getattr(messagelist[index.row()], elements[index.column()])
                    time , nano = time.split('.')
                    return QDateTime.fromTime_t(long(time)).toString(Qt.SystemLocaleLongDate)
                else:
                    return getattr(messagelist[index.row()], elements[index.column()])
            elif role == Qt.EditRole:
                #implement editable first row as editable
                raise  #editing not yet implemented
            elif role == Qt.ToolTipRole:
                return QWidget().tr('Right click for menu.')
                #triggers after hover for a second or so
                pass
            elif role == Qt.StatusTipRole:
                tip = str(self._messages.count()) + ' Messages'
                if self._messages.count() != self._messages.count(True):
                    tip = str(self._messages.count(True)) + ' of ' + tip
                tip = 'Displaying ' + tip
                return tip
                #NOTE this will only be called when you select multiple rows
        else:
            raise IndexError
    def get_selected_text(self, selection):
        return self._messages.get_selected_text(selection)

    def remove_rows(self, selection):
        return self._messages.remove_rows(self, selection)

    def insertRows(self, msg):
        AddDisplayRow = False
        if self._messages.test_conforms_to_filters(
                msg.msg, self._severity[msg.level], msg.name,
                str(msg.header.stamp.secs) + '.' + str(msg.header.stamp.nsecs),
                ', '.join(msg.topics), msg.file + ':' + msg.function + ':' +
                str(msg.line)):
            AddDisplayRow = True
        if AddDisplayRow:
            self.beginInsertRows(QModelIndex(),
                                 len(self._messages.get_message_list()),
                                 len(self._messages.get_message_list()))
        self._messages.add_message(msg.msg, self._severity[msg.level], msg.name,
                                   str(msg.header.stamp.secs) + '.' +
                                   str(msg.header.stamp.nsecs),
                                   ', '.join(msg.topics), msg.file +
                                   ':' + msg.function + ':' + str(msg.line))
        if AddDisplayRow:
            self.endInsertRows()

    def headerData(self, section, orientation, role=None):
        if role is None:
            role = Qt.DisplayRole
        if role == Qt.DisplayRole:
            if orientation == Qt.Horizontal:
                sections = self._messages.message_members()
                retval = sections[section][1:].capitalize()
                filtertext = self._messages._filters[section]._filtertext
                if filtertext is not None and len(filtertext) > 0:
                    if retval == 'Time':
                        mintime = filtertext[:filtertext.find(':')]
                        maxtime = filtertext[filtertext.find(':') + 1:]
                        qtime = QDateTime()
                        qtime.setTime_t(int(mintime))
                        retval += '(' + qtime.toLocalTime().toString()
                        qtime.setTime_t(int(maxtime))
                        retval += ':' + qtime.toLocalTime().toString() + ')'
                    else:
                        retval += '(' + filtertext + ')'
                return retval
            elif orientation == Qt.Vertical:
                return '#%d' % (section + 1)
        elif role == Qt.ToolTipRole:
            return 'Right click header for filter'
            #NOTE triggers after hover for a second or so
#        elif role == Qt.StatusTipRole:
#            #NOTE Doesn't seem to trigger would prefer this to the tooltip
#            return 'Double click a column header to filter the column'

        return None

    def sort(self, Ncol, order):
        self.layoutAboutToBeChanged.emit()
        self._messages.sort(Ncol, order)
        self.layoutChanged.emit()

    def delete_filter(self, index):
        self._messages.delete_filter(index)
        self.reset()

    def move_filter_down(self, index):
        self._messages.move_filter_down(index)
        self.reset()

    def move_filter_up(self, index):
        self._messages.move_filter_up(index)
        self.reset()

    def get_unique_col_data(self, index):
        return self._messages.get_unique_col_data(index)

    def get_data(self, row, col):
        return self._messages.get_data(row, col)

    def count(self, unfiltered=False):
        return self._messages.count(unfiltered)

    def set_filter(self, index, text):
        self._messages.set_filter(index, text)
        self.reset()

    def get_filter(self, index):
        return self._messages.get_filter(index)

    def message_members(self):
        return self._messages.message_members()

    def get_and(self):
        return self._messages.get_and()

    def get_or(self):
        return self._messages.get_or()

    def get_not(self):
        return self._messages.get_not()

    def save_to_file(self, filehandle):
        filehandle.write('rqt_console output file\n')
        for message in self._messages.get_message_list():
            filehandle.write(message.file_print())

    def open_from_file(self, filehandle):
        line = filehandle.readline()
        if line != 'rqt_console output file':
            while 1:
                line = filehandle.readline()
                if not line:
                    break
                self._messages.append_from_text(line)
            self.reset()
