#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rosgui.QtBindingHelper
import QtCore

class TreeModel(QtCore.QAbstractItemModel):

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

        def data(self, column):
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


    def __init__(self, parent=None):
        super(TreeModel, self).__init__(parent)
        self._clear()


    def _clear(self):
        self.rootItem = self.TreeItem([None])


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
        if role not in [QtCore.Qt.DisplayRole, QtCore.Qt.EditRole]:
            return None

        item = index.internalPointer()
        return item.data(index.column())


    def index(self, row, column, parent):
        if self.hasIndex(row, column, parent):
            parentItem = self.nodeFromIndex(parent)
            return self.createIndex(row, column, parentItem.child(row))

        return QtCore.QModelIndex()


    def parent(self, index):
        if index is not None and index.isValid():
            childItem = index.internalPointer()
            parentItem = childItem.parent()
            if parentItem is not None:
                return self.createIndex(parentItem.row(), 0, parentItem)

        return QtCore.QModelIndex()


    def rowCount(self, parent=None):
        if parent is None or not parent.isValid():
            parentItem = self.rootItem
        else:
            parentItem = parent.internalPointer()

        return parentItem.childCount()
