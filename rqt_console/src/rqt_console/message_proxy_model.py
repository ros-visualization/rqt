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

import qt_gui.qt_binding_helper  # @UnusedImport
from QtCore import Qt, qWarning
from QtGui import QBrush, QSortFilterProxyModel

from .filters.filter_collection import FilterCollection


class MessageProxyModel(QSortFilterProxyModel):
    """
    Provides sorting and filtering capabilities for the MessageDataModel.
    Filtering is based on subclassed Filters stored in the FilterCollections.
    """
    def __init__(self):
        super(MessageProxyModel, self).__init__()
        self._filters = []
        self.setDynamicSortFilter(True)
        self._show_highlighted_only = False

        self._exclude_filters = FilterCollection(self)
        self._highlight_filters = FilterCollection(self)

    # BEGIN Required implementations of QSortFilterProxyModel functions
    def filterAcceptsRow(self, sourcerow, sourceparent):
        """
        returns: True if the row does not match the exclude filters AND (_show_highlighted_only is False OR it matches the _highlight_filters, ''bool''
        OR
        returns: False if the row matches the exclude filters OR (_show_highlighted_only is True and it doesn't match the _highlight_filters, ''bool''
        """
        rowdata = []
        for index in range(self.sourceModel().columnCount()):
            rowdata.append(self.sourceModel().index(sourcerow, index, sourceparent).data())

        if self._exclude_filters.test_message_array(rowdata):
            return False
        if self._highlight_filters.count_enabled_filters() == 0:
            return True
        match = self._highlight_filters.test_message_array(rowdata)

        return not self._show_highlighted_only or match

    def data(self, index, role=None):
        """
        Sets colors of items based on highlight filters and severity type
        """
        messagelist = self.sourceModel()._messages.get_message_list()
        index = self.mapToSource(index)
        if index.row() >= 0 or index.row() < len(messagelist):
            if index.column() >= 0 or index.column() < messagelist[index.row()].count():
                if role == Qt.ForegroundRole and self._highlight_filters.count_enabled_filters() > 0:
                    if index.column() == 1:
                        data = index.data()
                        if data == 'Debug':
                            return QBrush(Qt.cyan)
                        elif data == 'Info':
                            return QBrush(Qt.darkCyan)
                        elif data == 'Warn':
                            return QBrush(Qt.yellow)
                        elif data == 'Error':
                            return QBrush(Qt.darkRed)
                        elif data == 'Fatal':
                            return QBrush(Qt.darkRed)
                    if not self._highlight_filters.test_message(messagelist[index.row()]):
                        return QBrush(Qt.gray)
        return self.sourceModel().data(index, role)
    # END Required implementations of QSortFilterProxyModel functions

    def handle_filters_changed(self):
        self.reset()

    def add_exclude_filter(self, newfilter):
        self._exclude_filters.append(newfilter)

    def add_highlight_filter(self, newfilter):
        self._highlight_filters.append(newfilter)

    def delete_exclude_filter(self, index):
        if index < self._exclude_filters.count() and index >= 0:
            del self._exclude_filters[index]
            self.reset()
            return True
        return False

    def delete_highlight_filter(self, index):
        if index < self._highlight_filters.count() and index >= 0:
            del self._highlight_filters[index]
            self.reset()
            return True
        return False

    def set_show_highlighted_only(self, show_highlighted_only):
        self._show_highlighted_only = show_highlighted_only
        self.reset()

    def save_to_file(self, filehandle):
        """
        Saves to an already open filehandle.
        :return: True if file write is successful, ''bool''
        OR
        :return: False if file write fails, ''bool''
        """
        try:
            filehandle.write(self.sourceModel()._messages.header_print())
            for index in range(self.rowCount()):
                row = self.mapToSource(self.index(index, 0)).row()
                filehandle.write(self.sourceModel()._messages.get_message_list()[row].file_print())
        except Exception as e:
            qWarning('File save failed: %s' % str(e))
            return False
        return True
