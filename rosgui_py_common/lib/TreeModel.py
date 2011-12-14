#!/usr/bin/env python

# Copyright (c) 2011, Dorian Scholz, TU Darmstadt
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the TU Darmstadt nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rosgui.QtBindingHelper #@UnusedImport
from QtCore import QAbstractItemModel, QModelIndex, Qt

class TreeModel(QAbstractItemModel):

    class TreeItem(object):
        def __init__(self, data, parent=None):
            self.parentItem = parent
            self.itemData = data
            self.childItems = []

        def appendChild(self, item):
            self.childItems.append(item)

        def child(self, row):
            return self.childItems[row]

        def childCount(self):
            return len(self.childItems)

        def columnCount(self):
            return len(self.itemData)

        def data(self, column, role=Qt.DisplayRole):
            if role not in [Qt.DisplayRole, Qt.EditRole]:
                return None
            try:
                return self.itemData[column]
            except IndexError:
                return None

        def parent(self):
            return self.parentItem

        def row(self):
            if self.parentItem is None:
                return 0
            return self.parentItem.childItems.index(self)


    def __init__(self, parent=None, column_names=None):
        super(TreeModel, self).__init__(parent)
        self._column_names = column_names or []
        self._clear()


    def _clear(self):
        self.rootItem = self.TreeItem(self._column_names)


    def nodeFromIndex(self, index=None):
        if index is not None and index.isValid():
            return index.internalPointer()
        else:
            return self.rootItem


    def columnCount(self, parent=None):
        if parent is None or not parent.isValid():
            return self.rootItem.columnCount()
        return parent.internalPointer().columnCount()


    def data(self, index, role):
        if not index.isValid():
            return None

        item = index.internalPointer()
        return item.data(index.column(), role)


    def headerData(self, section, orientation, role):
        if orientation != Qt.Horizontal:
            return None
        if role not in [Qt.DisplayRole, Qt.EditRole]:
            return None
        if section >= len(self._column_names):
            return None
        return self._column_names[section]


    def index(self, row, column, parent):
        if self.hasIndex(row, column, parent):
            parentItem = self.nodeFromIndex(parent)
            return self.createIndex(row, column, parentItem.child(row))

        return QModelIndex()


    def parent(self, index):
        if index is not None and index.isValid():
            childItem = index.internalPointer()
            parentItem = childItem.parent()
            if parentItem is not None:
                return self.createIndex(parentItem.row(), 0, parentItem)

        return QModelIndex()


    def rowCount(self, parent=None):
        if parent is None or not parent.isValid():
            parentItem = self.rootItem
        else:
            parentItem = parent.internalPointer()

        return parentItem.childCount()
