#!/usr/bin/env python

from __future__ import division
import os

from rosgui.QtBindingHelper import loadUi
from QtCore import Qt, QTimer, Slot
from QtGui import QDockWidget, QMenu, QTreeWidgetItem

import roslib
roslib.load_manifest('rosgui_topic')
import rospy
import TopicInfo
reload(TopicInfo) # force reload to update on changes during runtime


# main class inherits from the ui window class
class TopicWidget(QDockWidget):
    _column_names = ['topic', 'type', 'bandwidth', 'rate', 'value']

    def __init__(self, plugin, plugin_context):
        super(TopicWidget, self).__init__(plugin_context.main_window())
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'TopicWidget.ui')
        loadUi(ui_file, self)
        self._plugin = plugin

        self._current_topic_list = []
        self._topic_infos = {}
        self._tree_items = {}
        self._column_index = {}
        for column_name in self._column_names:
            self._column_index[column_name] = len(self._column_index)

        self.refresh_topics()
        # init and start update timer
        self._timer_refresh_topics = QTimer(self)
        self._timer_refresh_topics.timeout.connect(self.refresh_topics)
        self._timer_refresh_topics.start(1000)


    @Slot()
    def refresh_topics(self):
        # fill tree view
        topic_list = rospy.get_published_topics()
        if self._current_topic_list != topic_list:
            self._current_topic_list = topic_list

            # start new topic dict
            self.topics_tree_widget.clear()
            new_topic_infos = {}

            for topic_name, message_type in topic_list:
                # if topic is new
                if topic_name not in self._topic_infos:
                    # create new Topicinfo
                    topic_info = TopicInfo.TopicInfo(topic_name)
                    # if successful add it to the dict
                    if topic_info._topic_name:
                        new_topic_infos[topic_name] = topic_info
                else:
                    # if topic has been seen before, copy it to new dict and remove it form the old one
                    new_topic_infos[topic_name] = self._topic_infos[topic_name]
                    del self._topic_infos[topic_name]

                # if TopicInfo exist for topic, add it to tree view
                if topic_name in new_topic_infos:
                    self._recursive_create_widget_items(self.topics_tree_widget, topic_name, message_type, new_topic_infos[topic_name].message_class())

            # stop monitoring and delete non existing topics
            for topic_info in self._topic_infos.values():
                topic_info.stop_monitoring()
                del topic_info

            # change to new topic dict
            self._topic_infos = new_topic_infos


        for topic_info in self._topic_infos.values():
            if topic_info.monitoring:
                # update rate
                rate, _, _, _ = topic_info.get_hz()
                rate_text = '%1.2f' % rate if rate != None else 'unknown'

                # update bandwidth
                bytes_per_s, _, _, _ = topic_info.get_bw()
                if bytes_per_s is None:
                    bandwidth_text = 'unknown'
                elif bytes_per_s < 1000:
                    bandwidth_text = '%.2fB/s' % bytes_per_s
                elif bytes_per_s < 1000000:
                    bandwidth_text = '%.2fKB/s' % (bytes_per_s / 1000.)
                else:
                    bandwidth_text = '%.2fMB/s' % (bytes_per_s / 1000000.)

                # update values
                value_text = ''
                self.update_value(topic_info._topic_name, topic_info.last_message)

            else:
                rate_text = ''
                bandwidth_text = ''
                value_text = 'not monitored'

            self._tree_items[topic_info._topic_name].setText(self._column_index['rate'], rate_text)
            self._tree_items[topic_info._topic_name].setText(self._column_index['bandwidth'], bandwidth_text)
            self._tree_items[topic_info._topic_name].setText(self._column_index['value'], value_text)


        # resize columns
        for i in range(self.topics_tree_widget.columnCount()):
            self.topics_tree_widget.resizeColumnToContents(i)

        # limit width of value column
        current_width = self.topics_tree_widget.columnWidth(self._column_index['value'])
        self.topics_tree_widget.setColumnWidth(self._column_index['value'], min(150, current_width))


    def update_value(self, topic_name, message):
        if hasattr(message, '__slots__'):
            for slot_name in message.__slots__:
                self.update_value(topic_name + '/' + slot_name, getattr(message, slot_name))

        elif type(message) in (list, tuple) and (len(message) > 0) and hasattr(message[0], '__slots__'):
            for index, slot in enumerate(message):
                self.update_value(topic_name + '[%d]' % index, slot)

        else:
            if topic_name in self._tree_items:
                self._tree_items[topic_name].setText(self._column_index['value'], repr(message))


    def _extract_array_info(self, type_str):
        array_size = None
        if '[' in type_str and type_str[-1] == ']':
            type_str, array_size_str = type_str.split('[', 1)
            array_size_str = array_size_str[:-1]
            if len(array_size_str) > 0:
                array_size = int(array_size_str)
            else:
                array_size = 0

        return type_str, array_size


    def _recursive_create_widget_items(self, parent, topic_name, type_name, message):
        if parent is self.topics_tree_widget:
            # show full topic name with preceding namespace on toplevel item
            topic_text = topic_name
        else:
            topic_text = topic_name.split('/')[-1]
        item = QTreeWidgetItem(parent)
        item.setText(self._column_index['topic'], topic_text)
        item.setText(self._column_index['type'], type_name)
        item.setData(0, Qt.UserRole, topic_name)
        self._tree_items[topic_name] = item
        if hasattr(message, '__slots__') and hasattr(message, '_slot_types'):
            for slot_name, type_name in zip(message.__slots__, message._slot_types):
                self._recursive_create_widget_items(item, topic_name + '/' + slot_name, type_name, getattr(message, slot_name))

        else:
            base_type_str, array_size = self._extract_array_info(type_name)
            try:
                base_instance = roslib.message.get_message_class(base_type_str)()
            except ValueError:
                base_instance = None
            if array_size is not None and hasattr(base_instance, '__slots__'):
                for index in range(array_size):
                    self._recursive_create_widget_items(item, topic_name + '[%d]' % index, type_name, base_instance)


    @Slot('QPoint')
    def on_topics_tree_widget_customContextMenuRequested(self, pos):
        item = self.topics_tree_widget.itemAt(pos)
        if item is None:
            return

        menu = QMenu(self)
        actionToggleMonitoring = menu.addAction("Toggle Monitoring")
        action = menu.exec_(self.topics_tree_widget.mapToGlobal(pos))
        if action is actionToggleMonitoring:
            root_item = item
            while root_item.parent() is not None:
                root_item = root_item.parent()
            root_topic_name = root_item.data(0, Qt.UserRole)
            self._topic_infos[root_topic_name].toggle_monitoring()


    # override Qt's closeEvent() method to trigger _plugin unloading
    def closeEvent(self, event):
        for topic_info in self._topic_infos.values():
            topic_info.stop_monitoring()
        self._timer_refresh_topics.stop()
        event.ignore()
        self._plugin.deleteLater()
