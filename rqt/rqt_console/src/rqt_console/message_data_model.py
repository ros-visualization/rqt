import time
from PyQt4.QtCore import * #TODO figure out what i am supposed to import to use the "SIGNAL" keyword
from filtered_list import FilteredList

from QtCore import QAbstractTableModel, QByteArray, QMimeData, QModelIndex, Qt, Signal, QDateTime, pyqtSignal

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
            #TODO This implementation is not ideal. the names of 
            #the members of Message should be hidden at this level
                elements = ('_message', '_severity', '_node', '_time', '_topics', '_location')
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
                #triggers after hover for a second or so
                pass
            elif role == Qt.StatusTipRole:
                return 'To delete these rows press the delete key.'
                #NOTE this will be called when you select multiple rows
                pass
#        else:
#            print len(messagelist)
#            print index.row()
#            raise IndexError

    def remove_rows(self, selection):
        return self._messages.remove_rows(self, selection)

    def insertRows(self, msg):
        # ignore row and always append 
        # need to know BEFOREHAND how many rows will be added!
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
        #TODO this line needs to be prettier!
        #TODO This implementation is not ideal. 
        #the names of the members of Message should be hidden at this level
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
                if filtertext != '':
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
            return 'Double click on a header to filter the column'
            #NOTE triggers after hover for a second or so
            pass
        elif role == Qt.StatusTipRole:
            #NOTE Doesn't seem to trigger would prefer this to the tooltip
            return 'Double click a column header to filter the column'
            pass

        return None
    def sort(self, Ncol, order):
        self.emit(SIGNAL("layoutAboutToBeChanged()"))
        self._messages.sort(Ncol, order)
        self.emit(SIGNAL("layoutChanged()"))

    def delete_filter(self, index):
        self._messages.delete_filter(index)
        self.reset()

    def alter_filter_text(self, index, filtertext):
        self._messages.alter_filter_text(index, filtertext)
        self.reset()

    def move_filter_down(self, index):
        self._messages.move_filter_down(index)
        self.reset()

    def move_filter_up(self, index):
        self._messages.move_filter_up(index)
        self.reset()

    def get_unique_col_data(self, index):
        return self._messages.get_unique_col_data(index)
