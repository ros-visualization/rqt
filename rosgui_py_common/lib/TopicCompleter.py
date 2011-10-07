#!/usr/bin/env python
# -*- coding: utf-8 -*-
from rosgui.QtBindingHelper import import_from_qt
Slot = import_from_qt(['Slot'], 'QtCore')
QCompleter = import_from_qt(['QCompleter'], 'QtGui')

from TopicTreeModel import TopicTreeModel
from TreeModelCompleter import TreeModelCompleter

class TopicCompleter(TreeModelCompleter):

    def __init__(self, parent=None):
        super(TopicCompleter, self).__init__(parent)
        self.setModel(TopicTreeModel(parent))


    def update_topics(self):
        self.model().update_topics()


if __name__ == '__main__':
    import sys
    QApplication, QLineEdit, QTreeView = import_from_qt(['QApplication', 'QLineEdit', 'QTreeView'], 'QtGui')
    app = QApplication(sys.argv)
    l = QLineEdit()
    c = TopicCompleter(l)
    #c.setCompletionMode(QCompleter.InlineCompletion)
    l.setCompleter(c)

    treeView = QTreeView()
    treeView.setModel(c.model())
    treeView.header().hide()
    treeView.expandAll()
    treeView.move(0, 0)
    treeView.show()

    treeView2 = QTreeView()
    treeView2.setModel(c.completionModel())
    treeView2.header().hide()
    treeView2.expandAll()
    treeView2.move(300, 0)
    treeView2.show()

    l.move(600, 0)
    l.show()
    app.exec_()

