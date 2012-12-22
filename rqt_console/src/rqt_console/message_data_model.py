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

from message_list import MessageList

from python_qt_binding.QtCore import QAbstractTableModel, QDateTime, QModelIndex, Qt, qWarning
from python_qt_binding.QtGui import QIcon


class MessageDataModel(QAbstractTableModel):
    def __init__(self):
        super(MessageDataModel, self).__init__()
        self._messages = MessageList()

        self._time_format = 'hh:mm:ss.zzz (yyyy-MM-dd)'
        self._insert_message_queue = []
        self._paused = False
        self._message_limit = 20000
        self._error_icon = QIcon.fromTheme('dialog-error')
        self._warning_icon = QIcon.fromTheme('dialog-warning')
        self._info_icon = QIcon.fromTheme('dialog-information')

    # BEGIN Required implementations of QAbstractTableModel functions

    def rowCount(self, parent=None):
        return len(self._messages.get_message_list())

    def columnCount(self, parent=None):
        return self._messages.column_count()

    def data(self, index, role=None):
        if role is None:
            role = Qt.DisplayRole
        messagelist = self._messages.get_message_list()
        if index.row() >= 0 and index.row() < len(messagelist):
            if index.column() >= 0 and index.column() < messagelist[index.row()].count():
                if role == Qt.DisplayRole:
                    elements = self._messages.message_members()
                    if elements[index.column()] == '_time':
                        return self.timedata_to_timestring(messagelist[index.row()].time_in_seconds())
                    else:
                        return getattr(messagelist[index.row()], elements[index.column()])
                elif role == Qt.DecorationRole and index.column() == 0:
                    msgseverity = messagelist[index.row()].get_data(1)
                    if msgseverity in (self.tr('Debug'), self.tr('Info')):
                        return self._info_icon
                    elif msgseverity in (self.tr('Warn')):
                        return self._warning_icon
                    elif msgseverity in (self.tr('Error'), self.tr('Fatal')):
                        return self._error_icon
                elif role == Qt.ToolTipRole:
                    return self.tr('Right click for menu.')

    def headerData(self, section, orientation, role=None):
        if role is None:
            role = Qt.DisplayRole
        if role == Qt.DisplayRole:
            if orientation == Qt.Horizontal:
                sections = self._messages.message_members()
                retval = sections[section][1:].capitalize()
                return retval
            elif orientation == Qt.Vertical:
                return '#%d' % (section + 1)
    # END Required implementations of QAbstractTableModel functions

    def timestring_to_timedata(self, timestring):
        """
        Converts a time string in the format of _time_format into a string
        of format '(unix timestamp).(fraction of second)'
        :param timestring: formatted time string ''str''
        :returns: seconds and fractions thereof ''str''
        """
        timeval = QDateTime.fromString(timestring, self._time_format).toTime_t()
        return str(timeval) + '.' + timestring[9:12]   # append '.(msecs)'

    def timedata_to_timestring(self, timedata):
        """
        Converts a string in the format of '(unix timestamp).(fraction of second)'
        into a string of format _time_format
        :param timedata:  seconds and fractions thereof ''str''
        :returns: formatted time string''str''
        """
        sec, fraction = timedata.split('.')
        if len(fraction) < 3:
            raise RuntimeError('Malformed timestring in timedata_to_timestring()')
        micro = int(fraction[:3])
        return QDateTime.fromTime_t(long(sec)).addMSecs(micro).toString(self._time_format)

    def insert_rows(self, msgs):
        """
        Wraps the insert_row function to minimize gui notification calls
        """
        if len(msgs) == 0:
            return
        self.beginInsertRows(QModelIndex(), len(self._messages.get_message_list()), len(self._messages.get_message_list()) + len(msgs) - 1)
        for msg in msgs:
            self.insert_row(msg, False)
        self.endInsertRows()

        if len(self.get_message_list()) > self._message_limit:
            self.beginRemoveRows(QModelIndex(), 0, len(self.get_message_list()) - self._message_limit - 1)
            del self.get_message_list()[0:len(self.get_message_list()) - self._message_limit]
            self.endRemoveRows()

    def insert_row(self, msg, notify_model=True):
        if notify_model:
            self.beginInsertRows(QModelIndex(), len(self._messages.get_message_list()), len(self._messages.get_message_list()))
        self._messages.add_message(msg)
        if notify_model:
            self.endInsertRows()

    def remove_rows(self, rowlist):
        """
        :param rowlist: list of row indexes, ''list(int)''
        :returns: True if the indexes were removed successfully, ''bool''
        OR
        :returns: False if there was an exception removing the rows, ''bool''
        """
        if len(rowlist) == 0:
            if len(self.get_message_list()) > 0:
                try:
                    self.beginRemoveRows(QModelIndex(), 0, len(self.get_message_list()))
                    del self.get_message_list()[0:len(self.get_message_list())]
                    self.endRemoveRows()
                except:
                    return False
        else:
            rowlist = list(set(rowlist))
            rowlist.sort(reverse=True)
            dellist = [rowlist[0]]
            for row in rowlist[1:]:
                if dellist[-1] - 1 > row:
                    try:
                        self.beginRemoveRows(QModelIndex(), dellist[-1], dellist[0])
                        del self.get_message_list()[dellist[-1]:dellist[0] + 1]
                        self.endRemoveRows()
                    except:
                        return False
                    dellist = []
                dellist.append(row)
            if len(dellist) > 0:
                try:
                    self.beginRemoveRows(QModelIndex(), dellist[-1], dellist[0])
                    del self.get_message_list()[dellist[-1]:dellist[0] + 1]
                    self.endRemoveRows()
                except:
                    return False
        return True

    def get_selected_text(self, rowlist):
        """
        Returns an easily readable block of text for the currently selected rows
        :param rowlist: list of row indexes, ''list(int)''
        :returns: the text from those indexes, ''str''
        """
        text = None
        if len(rowlist) != 0:
            text = ''
            rowlist = list(set(rowlist))
            for row in rowlist:
                text += self.get_message_list()[row].pretty_print()
        return text

    def get_time_range(self, rowlist):
        """
        :param rowlist: a list of row indexes, ''list''
        :returns: a tuple of min and max times in a rowlist in '(unix timestamp).(fraction of second)' format, ''tuple(str,str)''
        """
        min_ = float("inf")
        max_ = float("-inf")
        for row in rowlist:
            item = self.get_message_list()[row].time_in_seconds()
            if float(item) > float(max_):
                max_ = item
            if float(item) < float(min_):
                min_ = item
        return min_, max_

    def get_unique_col_data(self, index, separate_topics=True):
        """
        :param index: column index, ''int''
        :param separate_topics: if true separates comma delimited strings into
        unique rows, ''bool''
        :returns: list of unique strings in the column, ''list[str]''
        """
        if index == 4 and separate_topics:
            unique_list = set()
            for topiclists in self._messages.get_unique_col_data(index):
                for item in topiclists.split(','):
                    unique_list.add(item.strip())
            return list(unique_list)
        return self._messages.get_unique_col_data(index)

    def get_severity_list(self):
        return [self.tr('Debug'), self.tr('Info'), self.tr('Warn'), self.tr('Error'), self.tr('Fatal')]

    def get_data(self, row, col):
        return self._messages.get_data(row, col)

    def message_members(self):
        return self._messages.message_members()

    def load_from_file(self, filehandle):
        """
        Saves to an already open filehandle.
        :returns: True if loaded successfully, ''bool''
        OR
        :returns: False if load fails, ''bool''
        """
        line = filehandle.readline()
        lines = []
        if line == self._messages.header_print():
            while 1:
                line = filehandle.readline()
                if not line:
                    break
                while line[-2] != "\"":
                    newline = filehandle.readline()
                    if not line:
                        qWarning('File does not appear to be a rqt_console message file: missing " at end of file')
                        return False
                    line = line + newline
                lines.append(line)
            self.beginInsertRows(QModelIndex(), len(self._messages.get_message_list()), len(self._messages.get_message_list()) + len(lines) - 1)
            for line in lines:
                try:
                    self._messages.append_from_text(line)
                except ValueError:
                    qWarning('File does not appear to be a rqt_console message file: File line is malformed %s' % line)
            self.endInsertRows()
            self._paused = True
            return True
        else:
            qWarning('File does not appear to be a rqt_console message file: missing file header.')
            return False

    def get_message_list(self, start_time=None, end_time=None):
        """
        :param start_time: time to start in timestamp form (including decimal
        fractions of a second is acceptable, ''unixtimestamp'' (Optional)
        :param end_time: time to end in timestamp form (including decimal
        fractions of a second is acceptable, ''unixtimestamp'' (Optional)
        :returns: list of messages in the time range ''list[message]''
        """
        if start_time is not None:
            return self._messages.get_messages_in_time_range(start_time, end_time)
        else:
            return self._messages.get_message_list()
