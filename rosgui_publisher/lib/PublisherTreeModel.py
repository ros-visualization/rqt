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
import threading

import roslib
roslib.load_manifest('rosgui_publisher')
import rospy

import rosgui.QtBindingHelper #@UnusedImport
from QtCore import Signal, qDebug, Qt
from QtGui import QStandardItem

import MessageTreeModel
reload(MessageTreeModel) # force reload to update on changes during runtime

class PublisherTreeModel(MessageTreeModel.MessageTreeModel):
    _column_names = ['topic', 'type', 'rate', 'enabled', 'expression']
    item_value_changed = Signal(int, str, str, str, object)
    topic_name_role = Qt.UserRole

    def __init__(self, parent=None):
        super(PublisherTreeModel, self).__init__(parent, self._column_names)
        self._column_index = {}
        for column_name in self._column_names:
            self._column_index[column_name] = len(self._column_index)
        self.clear()

        self._item_change_lock = threading.Lock()
        self.itemChanged.connect(self.handle_item_changed)

    def clear(self):
        super(PublisherTreeModel, self).clear()
        self.setHorizontalHeaderLabels(self._column_names)


    def _get_toplevel_items(self, index_list):
        items = [self.itemFromIndex(index) for index in index_list]
        uniqueItems = {}
        for item in items:
            while item.parent() is not None:
                item = item.parent()
            if item.row() not in uniqueItems:
                uniqueItems[item.row()] = item
        return uniqueItems.values()


    def get_publisher_ids(self, index_list):
        return [item.publisher_id for item in self._get_toplevel_items(index_list)]


    def remove_items_with_parents(self, index_list):
        for item in self._get_toplevel_items(index_list):
            self.removeRow(item.row())


    def handle_item_changed(self, item):
        if not self._item_change_lock.acquire(False):
            #qDebug('PublisherTreeModel.handle_item_changed(): could not acquire lock')
            return
        # lock has been acquired
        topic_name = item.data(self.topic_name_role)
        column_name = self._column_names[item.column()]
        new_value = item.text().strip()
        #qDebug('PublisherTreeModel.handle_item_changed(): %s, %s, %s' % (topic_name, column_name, new_value))

        self.item_value_changed.emit(item.publisher_id, topic_name, column_name, new_value, item.setText)

        # release lock
        self._item_change_lock.release()


    def _recursive_create_items(self, parent, topic_name, type_name, message, publisher_id, expressions):
        if parent is None:
            parent = self
            # show full topic name with preceding namespace on toplevel item
            topic_text = topic_name
        else:
            topic_text = topic_name.split('/')[-1]
            if '[' in topic_text:
                topic_text = topic_text[topic_text.index('['):]

        items = dict((name, QStandardItem()) for name in self._column_names)
        for item in items.values():
            item.publisher_id = publisher_id
            item.setData(topic_name, self.topic_name_role)

        items['topic'].setText(topic_text)
        items['type'].setText(type_name)

        if hasattr(message, '__slots__') and hasattr(message, '_slot_types'):
            for slot_name, type_name in zip(message.__slots__, message._slot_types):
                self._recursive_create_items(items['topic'], topic_name + '/' + slot_name, type_name, getattr(message, slot_name), publisher_id, expressions)

        elif type(message) in (list, tuple) and (len(message) > 0) and hasattr(message[0], '__slots__'):
            type_name = type_name.split('[', 1)[0]
            for index, slot in enumerate(message):
                self._recursive_create_items(items['topic'], topic_name + '[%d]' % index, type_name, slot, publisher_id, expressions)

        else:
            if topic_name in expressions:
                items['expression'].setText(expressions[topic_name])
            elif not hasattr(message, '__slots__'):
                items['expression'].setText(repr(message))

        parent.appendRow([items[name] for name in self._column_names])
        return items
