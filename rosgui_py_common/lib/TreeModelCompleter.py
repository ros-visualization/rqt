#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rosgui.QtBindingHelper
from QtGui import QCompleter

class TreeModelCompleter(QCompleter):
    separator = '/'

    def __init__(self, parent=None):
        super(TreeModelCompleter, self).__init__(parent)


    def splitPath(self, path):
        path = path.lstrip(self.separator)
        path_list = path.split(self.separator)
        return path_list


    def pathFromIndex(self, index):
        item = self.model().nodeFromIndex(index)
        path_list = []
        while item.parent() is not None:
            path_list.insert(0, item.data(0))
            item = item.parent()
        path = self.separator + self.separator.join(path_list)
        return path
