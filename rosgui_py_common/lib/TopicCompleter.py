#!/usr/bin/env python

import TopicTreeModel
reload(TopicTreeModel) # force reload to update on changes during runtime
import TreeModelCompleter
reload(TreeModelCompleter) # force reload to update on changes during runtime

class TopicCompleter(TreeModelCompleter.TreeModelCompleter):

    def __init__(self, parent=None):
        super(TopicCompleter, self).__init__(parent)
        self.update_topics()


    def update_topics(self):
        self.setModel(TopicTreeModel.TopicTreeModel())


if __name__ == '__main__':
    import sys
    import rosgui.QtBindingHelper #@UnusedImport
    from QtGui import QApplication, QComboBox, QMainWindow, QVBoxLayout, QWidget, QLineEdit, QTreeView
    app = QApplication(sys.argv)
    mw = QMainWindow()
    widget = QWidget(mw)
    layout = QVBoxLayout(widget)

    edit = QLineEdit()
    edit_completer = TopicCompleter(edit)
    #edit_completer.setCompletionMode(QCompleter.InlineCompletion)
    edit.setCompleter(edit_completer)

    combo = QComboBox()
    combo.setEditable(True)
    combo_completer = TopicCompleter(combo)
    #combo_completer.setCompletionMode(QCompleter.InlineCompletion)
    combo.lineEdit().setCompleter(combo_completer)

    model_tree = QTreeView()
    model_tree.setModel(combo_completer.model())
    model_tree.expandAll()
    for column in range(combo_completer.model().columnCount()):
        model_tree.resizeColumnToContents(column)

    completion_tree = QTreeView()
    completion_tree.setModel(combo_completer.completionModel())
    completion_tree.expandAll()
    for column in range(combo_completer.completionModel().columnCount()):
        completion_tree.resizeColumnToContents(column)

    layout.addWidget(model_tree)
    layout.addWidget(completion_tree)
    layout.addWidget(edit)
    layout.addWidget(combo)
    layout.setStretchFactor(model_tree, 1)
    widget.setLayout(layout)
    mw.setCentralWidget(widget)

    mw.move(300, 0)
    mw.resize(800, 900)
    mw.show()
    app.exec_()
