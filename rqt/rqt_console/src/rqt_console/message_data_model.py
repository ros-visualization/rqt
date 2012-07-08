import time
#from filtered_list import FilteredList
from message_list import MessageList

from QtCore import QAbstractTableModel, QByteArray, QDateTime, qDebug, QMimeData, QModelIndex, Qt, qWarning, Signal
from QtGui import QWidget

class MessageDataModel(QAbstractTableModel):

    def __init__(self, editable=False):
        QAbstractTableModel.__init__(self)
        self._messages = MessageList()

        self._severity = {1: 'Debug', 2: 'Info', 4:'Warn', 8:'Error', 16: 'Fatal'}
        
        self._header_filter_text = []
        for item in self._messages.message_members():
            self._header_filter_text.append('')

    def rowCount(self, parent=None):
        return len(self._messages.get_message_list())

    def columnCount(self, parent=None):
        return self._messages.columnCount()

    def data(self, index, role=None):
        if role is None:
            role = Qt.DisplayRole
        messagelist = self._messages.get_message_list()
        if index.row() >= 0 and index.row() < len(messagelist):
            if index.column() < 0 or index.column() >= messagelist[index.row()].CountElements():
                qWarning('Message Col Index out of bounds %s' % index.col())
                raise IndexError
            if role == Qt.DisplayRole:
                elements = self._messages.message_members()
                if elements[index.column()] == '_time':
                    time = getattr(messagelist[index.row()], elements[index.column()])
                    time, micro = time.split('.')
                    return QDateTime.fromTime_t(long(time)).addMSecs(int(micro[:3])).toString('hh:mm:ss.zzz (yyyy-MM-dd)')
                else:
                    return getattr(messagelist[index.row()], elements[index.column()])
            elif role == Qt.ToolTipRole:
                return QWidget().tr('Right click for menu.')
                pass
        else:
            qWarning('Message Row Index out of bounds %s' % index.row())
            raise IndexError

    def insertRows(self, msg):
        self.beginInsertRows(QModelIndex(),
                             len(self._messages.get_message_list()),
                             len(self._messages.get_message_list()))
        self._messages.add_message(msg.msg, self._severity[msg.level], msg.name,
                                   str(msg.header.stamp.secs) + '.' +
                                   str(msg.header.stamp.nsecs),
                                   ', '.join(msg.topics), msg.file +
                                   ':' + msg.function + ':' + str(msg.line))
        self.endInsertRows()

    def headerData(self, section, orientation, role=None):
        if role is None:
            role = Qt.DisplayRole
        if role == Qt.DisplayRole:
            if orientation == Qt.Horizontal:
                sections = self._messages.message_members()
                retval = sections[section][1:].capitalize() 
                if self._header_filter_text[section] != '':
                    retval += ' (' + self._header_filter_text[section] + ')'
#TODO readd after filters are in proxy                
#                filtertext = self._messages._filters[section]._filtertext
#                if filtertext is not None and len(filtertext) > 0:
#                    if retval == 'Time':
#                        mintime = filtertext[:filtertext.find(':')]
#                        maxtime = filtertext[filtertext.find(':') + 1:]
#                        qtime = QDateTime()
#                        qtime.setTime_t(int(mintime))
#                        retval += '(' + qtime.toLocalTime().toString()
#                        qtime.setTime_t(int(maxtime))
#                        retval += ':' + qtime.toLocalTime().toString() + ')'
#                    else:
#                        retval += '(' + filtertext + ')'
                return retval
            elif orientation == Qt.Vertical:
                return '#%d' % (section + 1)
        return None

    def remove_rows(self, rowlist):
        if len(rowlist) == 0:
            if len(self.get_message_list()) > 0:
                self.beginRemoveRows(QModelIndex(), 0, len(self.get_message_list()) )
                del self.get_message_list()[0:len(self.get_message_list())]
                self.endRemoveRows()
        else:
            rowlist = list(set(rowlist))
            rowlist.sort(reverse=True)
            for row in rowlist:
                self.beginRemoveRows(QModelIndex(), row, row)
                del self.get_message_list()[row]
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
            item = self.get_message_list()[row]._time 
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

    def get_message_list(self):
        return self._messages.get_message_list()

    def set_header_text(self, index, text):
        if index >= 0 and index < self._messages.message_members():
            self._header_filter_text[index] = text
