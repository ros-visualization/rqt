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

from message import Message
from message_list import MessageList
from QtGui import QBrush, QSortFilterProxyModel
from QtCore import QDateTime, Qt, QVariant, qWarning

from message_filter import MessageFilter
from filter_collection import FilterCollection

class MessageProxyModel(QSortFilterProxyModel):
    """
    Provides sorting and filtering capabilities for the MessageDataModel.
    Filtering is based on standard boolean operations. Base units in boolean
    operations are considered true if they exist in the string they are applied to.
    """
    def __init__(self):
        super(MessageProxyModel, self).__init__()
        self._filters = []
        self.setDynamicSortFilter(True)
        self._show_highlighted_only = False

        self._exclude_filters = FilterCollection(self)
        self._highlight_filters = FilterCollection(self)
    
    def filterAcceptsRow(self, sourcerow, sourceparent):
        rowdata = []
        for index in range(self.sourceModel().columnCount()):
            rowdata.append(self.sourceModel().index(sourcerow, index, sourceparent).data())
        
        if self._exclude_filters.test_message_array(rowdata):
            return False
        match = self._highlight_filters.test_message_array(rowdata)
        return not self._show_highlighted_only or match

#TODO THIS FUNCTION IS FOR TESTING DELETE BEFORE COMMIT
#    def lessThan_(self, left, right):
#        l = left.data(QtCore.Qt.DisplayRole).toDouble()
#        r = right.data(QtCore.Qt.DisplayRole).toDouble()
#        if not l[1] and not r[1]:
#            return QtGui.QSortFilterProxyModel.lessThan(self, left, right)
#        if l[1] ^ r[1]: return l[1]
#        return l[0] < r[0]

# TODO If uncommented this function breaks sorting. figure out why
#    def data(self, index, role=None):
#        messagelist = self.sourceModel()._messages.get_message_list()
#        if index.row() >= 0 and index.row() < len(messagelist):
#            if index.column() >= 0 and index.column() < messagelist[index.row()].count():
#                if role == Qt.ForegroundRole and self._highlight_filters.count_enabled_filters() > 0:
#                    match = self._highlight_filters.message_test(messagelist[index.row()])
#                    if not match:
#                        return QBrush(Qt.gray)
#        return self.sourceModel().data(index, role)

    def _check_special_chars(self, text):
        """
        Returns true if text contains a '(' or the variables _and, _or, _not
        """
        if text.find('(') != -1 or text.find(self._and) != -1 or text.find(self._or) != -1 or text.find(self._not) != -1:
            return True
        return False
    
    def filters_changed_handler(self):
        """
        This callback
        """
        # this should be called whenever the filters change
        self.reset()
    
    def filter_deleted(self):
        #this function should emit a signal for the console_widget so that it
        #can delete the deleted row
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
        self._show_highlighted_only = not show_highlighted_only
#        print 'show_highlighted_only: ', self._show_highlighted_only
        self.reset()

