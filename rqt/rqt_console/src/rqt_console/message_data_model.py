import time
from message_list import MessageList

from QtCore import QAbstractTableModel, QDateTime, qDebug, QModelIndex, Qt, qWarning, Signal
from QtGui import QWidget

class MessageDataModel(QAbstractTableModel):
    def __init__(self, editable=False):
        super(MessageDataModel, self).__init__()
        QAbstractTableModel.__init__(self)
        self._messages = MessageList()

        self._time_format = 'hh:mm:ss.zzz (yyyy-MM-dd)'
        self._header_filter_text = []
        for item in self._messages.message_members():
            self._header_filter_text.append('')
        self._insert_message_queue = []
        self._paused = False

    # BEGIN Required QAbstractTableModel functions
    def rowCount(self, parent=None):
        return len(self._messages.get_message_list())

    def columnCount(self, parent=None):
        return self._messages.column_count()

    def data(self, index, role=None):
        if role is None:
            role = Qt.DisplayRole
        messagelist = self._messages.get_message_list()
        if index.row() >= 0 and index.row() < len(messagelist):
            if index.column() < 0 or index.column() >= messagelist[index.row()].count():
                qWarning(self.tr('Message Column Index out of bounds %s' % index.col()))
                raise IndexError
            if role == Qt.DisplayRole:
                elements = self._messages.message_members()
                if elements[index.column()] == '_time':
                    return self.timedata_to_timestring(messagelist[index.row()].time_in_seconds())
                else:
                    return getattr(messagelist[index.row()], elements[index.column()])
            elif role == Qt.ToolTipRole:
                return self.tr('Right click for menu.')
        else:
            qWarning(self.tr('Message Row Index out of bounds %s' % index.row()))
            raise IndexError

    def headerData(self, section, orientation, role=None):
        if role is None:
            role = Qt.DisplayRole
        if role == Qt.DisplayRole:
            if orientation == Qt.Horizontal:
                sections = self._messages.message_members()
                retval = sections[section][1:].capitalize() 
                if self._header_filter_text[section] != '':
                    retval += '*'
                return retval
            elif orientation == Qt.Vertical:
                return '#%d' % (section + 1)
        elif role == Qt.ToolTipRole:
            if self._header_filter_text[section] != '':
                return self.tr('Filter: ') + self._header_filter_text[section]
            else:
                return self.tr('Column not filtered. \nA "*" will indicate a filtered column.')
        return None
    # END Required QAbstractTableModel functions

    def timestring_to_timedata(self, timestring):
        timeval = QDateTime.fromString(timestring,self._time_format).toTime_t()
        return str(timeval) + '.' + timestring[9:12]   # append '.(msecs)'

    def timedata_to_timestring(self, timedata):
        sec, fraction = timedata.split('.')
        if len(fraction) < 3:
            raise RuntimeError(self.tr('Malformed timestring in timedata_to_timestring()'))
        micro = int(fraction[:3])
        return QDateTime.fromTime_t(long(sec)).addMSecs(micro).toString(self._time_format)

    def insert_rows(self, msgs):
        if len(msgs) == 0:
            return
        self.beginInsertRows(QModelIndex(), len(self._messages.get_message_list()), len(self._messages.get_message_list()) + len(msgs)-1)
        for msg in msgs:
            self.insert_row(msg,False)
        self.endInsertRows()

    def insert_row(self, msg, notify_model=True):
        if notify_model:
            self.beginInsertRows(QModelIndex(),
                                 len(self._messages.get_message_list()),
                                 len(self._messages.get_message_list()))
        self._messages.add_message(msg)
        if notify_model:
            self.endInsertRows()


    def remove_rows(self, rowlist):
        if len(rowlist) == 0:
            if len(self.get_message_list()) > 0:
                self.beginRemoveRows(QModelIndex(), 0, len(self.get_message_list()) )
                del self.get_message_list()[0:len(self.get_message_list())]
                self.endRemoveRows()
        else:
            rowlist = list(set(rowlist))
            rowlist.sort(reverse=True)
            dellist = [rowlist[0]]
            for row in rowlist[1:]:
                if dellist[-1] - 1 > row:
                    self.beginRemoveRows(QModelIndex(), dellist[-1], dellist[0])
                    del self.get_message_list()[dellist[-1]:dellist[0]+1]
                    self.endRemoveRows()
                    dellist = []
                dellist.append(row)
            if len(dellist) > 0:
                self.beginRemoveRows(QModelIndex(), dellist[-1], dellist[0])
                del self.get_message_list()[dellist[-1]:dellist[0]+1]
                self.endRemoveRows()
        return True

    def get_selected_text(self, rowlist):
        text = None
        if len(rowlist) != 0:
            text = ''
            rowlist = list(set(rowlist))
            for row in rowlist:
                text += self.get_message_list()[row].pretty_print()
        return text

    def get_time_range(self, rowlist):
        min_ = float("inf")
        max_ = float("-inf")
        for row in rowlist:
            item = self.get_message_list()[row].time_in_seconds() 
            if float(item) > float(max_):
                max_ = item
            if float(item) < float(min_):
                min_ = item
        return min_, max_

    def get_unique_col_data(self, index):
        return self._messages.get_unique_col_data(index)

    def get_data(self, row, col):
        return self._messages.get_data(row, col)

    def message_members(self):
        return self._messages.message_members()

    def save_to_file(self, filehandle):
        try:
            filehandle.write(self._messages.header_print())
            for message in self._messages.get_message_list():
                filehandle.write(message.file_print())
            return True
        except:
            qWarning(self.tr('File save failed.'))
            return False
    def load_from_file(self, filehandle):
        line = filehandle.readline()
        if line == self._messages.header_print():
            while 1:
                line = filehandle.readline()
                if not line:
                    break
                self._messages.append_from_text(line)
            self.reset()
            return True
        else:
            qWarning(self.tr('File does not appear to be a rqt_console message file.'))
            return False

    def get_message_list(self):
        return self._messages.get_message_list()

    def set_header_text(self, index, text):
        if index >= 0 and index < self._messages.message_members():
            self._header_filter_text[index] = text
