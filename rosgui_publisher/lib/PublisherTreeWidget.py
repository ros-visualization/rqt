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

import roslib
roslib.load_manifest('rosgui_publisher')

import rosgui.QtBindingHelper #@UnusedImport
from QtCore import Signal, Slot, qDebug, QMimeData, QModelIndex, Qt
from QtGui import QDrag, QHeaderView, QIcon, QMenu, QTreeView

import PublisherTreeModel
reload(PublisherTreeModel) # force reload to update on changes during runtime

class PublisherTreeWidget(QTreeView):
    remove_publisher = Signal(int)
    publish_once = Signal(int)

    def __init__(self, parent=None):
        super(PublisherTreeWidget, self).__init__(parent)
        self.setDragEnabled(True)
        self.setModel(PublisherTreeModel.PublisherTreeModel())
        self.sortByColumn(0, Qt.AscendingOrder)

        self.header().setResizeMode(QHeaderView.ResizeToContents)
        self.header().setContextMenuPolicy(Qt.CustomContextMenu)
        self.header().customContextMenuRequested.connect(self.handle_header_view_customContextMenuRequested)

        self.customContextMenuRequested.connect(self.handle_customContextMenuRequested)


    @Slot()
    def remove_selected_publishers(self):
        publisher_ids = self.model().get_publisher_ids(self.selectedIndexes())
        for publisher_id in publisher_ids:
            self.remove_publisher.emit(publisher_id)
        self.model().remove_items_with_parents(self.selectedIndexes())


    def startDrag(self, supportedActions):
        index = self.currentIndex()
        if not index.isValid():
            return

        item = self.model().itemFromIndex(index)
        path = getattr(item, '_path', None)
        if path is None:
            qDebug('PublisherTreeWidget.startDrag(): no _path set on item %s' % item)
            return
        #qDebug('PublisherTreeWidget.startDrag(): %s' % item._path)

        data = QMimeData()
        data.setText(item._path)

        drag = QDrag(self)
        drag.setMimeData(data)
        drag.exec_()


    @Slot('QPoint')
    def handle_customContextMenuRequested(self, pos):
        indeces = self.selectedIndexes()
        if indeces == QModelIndex():
            return

        # show context menu
        menu = QMenu(self)
        action_remove_publisher = menu.addAction(QIcon.fromTheme('remove'), "Remove Publisher")
        action_publish_once = menu.addAction(QIcon.fromTheme('media-playback-start'), "Publish Once")
        action_item_expand = menu.addAction(QIcon.fromTheme('zoom-in'), "Expand All Children")
        action_item_collapse = menu.addAction(QIcon.fromTheme('zoom-out'), "Collapse All Children")
        action = menu.exec_(self.mapToGlobal(pos))

        # evaluate user action
        if action is action_remove_publisher:
            self.remove_selected_publishers()

        elif action is action_publish_once:
            for publisher_id in self.model().get_publisher_ids(indeces):
                self.publish_once.emit(publisher_id)

        elif action in (action_item_expand, action_item_collapse):
            expanded = (action is action_item_expand)
            def recursive_set_expanded(index):
                if index != QModelIndex():
                    self.setExpanded(index, expanded)
                    recursive_set_expanded(index.child(0, 0))
            for index in indeces:
                recursive_set_expanded(index)


    @Slot('QPoint')
    def handle_header_view_customContextMenuRequested(self, pos):

        # create context menu
        menu = QMenu(self)

        action_toggle_auto_resize = menu.addAction('Auto-Resize')
        action_toggle_auto_resize.setCheckable(True)
        auto_resize_flag = (self.header().resizeMode(0) == QHeaderView.ResizeToContents)
        action_toggle_auto_resize.setChecked(auto_resize_flag)

        action_toggle_sorting = menu.addAction('Sorting')
        action_toggle_sorting.setCheckable(True)
        action_toggle_sorting.setChecked(self.isSortingEnabled())

        # show menu
        action = menu.exec_(self.header().mapToGlobal(pos))

        # evaluate user action
        if action is action_toggle_auto_resize:
            if auto_resize_flag:
                self.header().setResizeMode(QHeaderView.Interactive)
            else:
                self.header().setResizeMode(QHeaderView.ResizeToContents)

        elif action is action_toggle_sorting:
            self.setSortingEnabled(not self.isSortingEnabled())
