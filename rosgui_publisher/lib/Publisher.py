#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division
import os, sys, inspect
from math import * #@UnusedWildImport so math functions can be used in the custom expressions
from random import random, randint, gauss

from rosgui.QtBindingHelper import import_from_qt, loadUi
Qt, QTimer, QSignalMapper, Slot, qDebug, qWarning = import_from_qt(['Qt', 'QTimer', 'QSignalMapper', 'Slot', 'qDebug', 'qWarning'], 'QtCore')
QDockWidget, QTreeWidgetItem, QMenu = import_from_qt(['QDockWidget', 'QTreeWidgetItem', 'QMenu'], 'QtGui')

import roslib
roslib.load_manifest('rosgui_publisher')
import rospy

# main class inherits from the ui window class
class Publisher(QDockWidget):
    column_names_ = ['topic', 'type', 'rate', 'enabled', 'expression']
    message_package_names_ = ['actionlib_msgs', 'diagnostic_msgs', 'geometry_msgs', 'nav_msgs', 'sensor_msgs', 'stereo_msgs', 'test_common_msgs', 'trajectory_msgs', 'visualization_msgs', 'std_msgs']


    def __init__(self, parent, plugin_context):
        QDockWidget.__init__(self, plugin_context.main_window())
        self.setObjectName('Publisher')

        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'Publisher.ui')
        loadUi(ui_file, self)

        self.column_index = {}
        for column_name in self.column_names_:
            self.column_index[column_name] = len(self.column_index)
        self.counter_ = 0
        self.publishers_ = {}
        self.id_counter_ = 0
        self.message_classes_ = {}
        for message_package_name in self.message_package_names_:
            self.add_message_package(message_package_name)
        
        self.signal_mapper_ = QSignalMapper(self)
        self.signal_mapper_.mapped[int].connect(self.timeout)
        
        self.publishers_tree_widget.itemChanged.connect(self.publishers_tree_widget_itemChanged)
        self.update_comboType()

        # add our self to the main window
        plugin_context.main_window().addDockWidget(Qt.RightDockWidgetArea, self)


    def add_message_package(self, package_name):
        try:
            package_dir = roslib.packages.get_pkg_dir(package_name)
        except Exception:
            qDebug('Publisher.add_message_package(%s): package dir not found' % package_name)
            return
        package_dir = os.path.join(package_dir, 'src')
        sys.path.append(package_dir)
        try:
            __import__(package_name + '.msg')
            package_module = sys.modules[package_name + '.msg']
        except Exception:
            qDebug('failed to import %s' % (package_name + '.msg'))
            return
        message_types = inspect.getmembers(package_module, inspect.isclass)
        if len(message_types) == 0:
            qDebug('Publisher.add_message_package(%s): no message classes found in %s' % (package_name, package_module))
        for type_name, message_type in message_types:
            self.message_classes_['%s/%s' % (package_name, type_name)] = message_type
    
    
    def update_comboType(self):
        self.type_combo_box.clear()
        self.type_combo_box.addItems(sorted(self.message_classes_.keys()))


    def check_valid_message_type(self, type_str):
        if not self.message_classes_.has_key(type_str):
            package_name = type_str.split('/', 1)[0]
            self.add_message_package(package_name)
            if not self.message_classes_.has_key(type_str):
                qDebug('Publisher.on_add_publisher_button_clicked(): failed to resolve message type "%s"' % type_str)
                return False
            self.update_comboType()
        return True


    @Slot()
    def on_add_publisher_button_clicked(self, publisher_info = {}):
        # create publisher info
        publisher_info['topic_name'] = str(self.topic_combo_box.currentText())
        publisher_info['type_name'] = str(self.type_combo_box.currentText())
        publisher_info['rate'] = float(self.frequency_combo_box.currentText())
        publisher_info['enabled'] = True
        self._add_publisher(publisher_info)


    def _add_publisher(self, publisher_info):
        publisher_info['publisher_id'] = self.id_counter_
        self.id_counter_ += 1
        publisher_info['counter'] = 0
        publisher_info['enabled'] = publisher_info.get('enabled', False)
        publisher_info['expressions'] = publisher_info.get('expressions', {})
        
        if not self.check_valid_message_type(publisher_info['type_name']):
            return
        publisher_info['type'] = self.message_classes_[publisher_info['type_name']]

        # create publisher and timer
        publisher_info['publisher'] = rospy.Publisher(publisher_info['topic_name'], publisher_info['type'])
        publisher_info['timer'] = QTimer(self)
        
        # add publisher info to publishers_ dict and create signal mapping  
        self.publishers_[publisher_info['publisher_id']] = publisher_info
        self.signal_mapper_.setMapping(publisher_info['timer'], publisher_info['publisher_id'])
        publisher_info['timer'].timeout.connect(self.signal_mapper_.map)
        if publisher_info['enabled']:
            publisher_info['timer'].start(int(1000.0 / publisher_info['rate']))
        
        # recursively create widget items for the message's slots 
        top_level_item = self._recursive_create_widget_items(None, publisher_info['topic_name'], publisher_info['type']._type, publisher_info['type'](), publisher_info['publisher_id'], publisher_info['expressions'])
        
        # fill tree widget columns of top level item
        top_level_item.setText(self.column_index['enabled'], str(publisher_info['enabled']))
        top_level_item.setText(self.column_index['rate'], str(publisher_info['rate']))
        publisher_info['widgetItem'] = top_level_item
        
        # add top level item to tree widget
        self.publishers_tree_widget.addTopLevelItem(top_level_item)
        
        # resize columns
        self.publishers_tree_widget.expandAll()
        for i in range(self.publishers_tree_widget.columnCount()):
            self.publishers_tree_widget.resizeColumnToContents(i)


    def _recursive_create_widget_items(self, parent, topic_name, type_name, message, publisher_id, expressions):
        if parent == None:
            # show full topic name with preceding namespace on toplevel item
            topic_text = topic_name
        else:
            topic_text = topic_name.split('/')[-1]
        item = QTreeWidgetItem(parent)
        item.setText(self.column_index['topic'], topic_text)
        item.setText(self.column_index['type'], type_name)
        if expressions.has_key(topic_name):
            item.setText(self.column_index['expression'], expressions[topic_name])
        elif not hasattr(message, '__slots__'):
            item.setText(self.column_index['expression'], repr(message))
        item.setFlags(item.flags() | Qt.ItemIsEditable)
        item.setData(0, Qt.UserRole, topic_name)
        item.publisher_id = publisher_id
        if hasattr(message, '__slots__') and hasattr(message, '_slot_types'):
            for slot_name, type_name in zip(message.__slots__, message._slot_types):
                self._recursive_create_widget_items(item, topic_name + '/' + slot_name, type_name, getattr(message, slot_name), publisher_id, expressions)
        return item


    @Slot('QTreeWidgetItem*', int)
    def publishers_tree_widget_itemChanged(self, item, column):
        column_name = self.column_names_[column]
        new_value = str(item.text(column))
        qDebug('Publisher.on_treePublishers_itemChanged(): %s : %s' % (column_name, new_value))
        if not hasattr(item, 'publisher_id'):
            qDebug('Publisher.on_treePublishers_itemChanged(): no publisher_id found in: %s' % (item))
        else:
            publisher_info = self.publishers_[item.publisher_id]
            
            if column_name == 'enabled':
                publisher_info['enabled'] = (new_value and new_value.lower() in ['1', 'true', 'yes'])
                qDebug('Publisher.on_treePublishers_itemChanged(): %s enabled: %s' % (publisher_info['topic_name'], publisher_info['enabled']))
                if publisher_info['enabled']:
                    publisher_info['timer'].start(int(1000.0 / publisher_info['rate']))
                else:
                    publisher_info['timer'].stop()
                
            elif column_name == 'rate':
                publisher_info['rate'] = float(new_value)
                qDebug('Publisher.on_treePublishers_itemChanged(): %s rate changed: %s' % (publisher_info['topic_name'], publisher_info['rate']))
                if publisher_info['enabled']:
                    publisher_info['timer'].stop()
                    publisher_info['timer'].start(int(1000.0 / publisher_info['rate']))
                    
            elif column_name == 'expression':
                topic_name = str(item.data(0, Qt.UserRole))
                publisher_info['expressions'][topic_name] = new_value
                qDebug('Publisher.on_treePublishers_itemChanged(): %s expression: %s' % (topic_name, new_value))
    
    
    def fill_message_slots(self, message, topic_name, expressions, i):
        if not hasattr(message, '__slots__'):
            return
        for slot_name in message.__slots__:
            slot_key = topic_name + '/' + slot_name
            
            # if no expression exists for this slot_key, continue with it's child slots
            if not expressions.has_key(slot_key):
                self.fill_message_slots(getattr(message, slot_name), slot_key, expressions, i)
                continue

            expression = expressions[slot_key]
            if len(expression) == 0:
                continue
            
            # get slot type
            slot = getattr(message, slot_name)
            if hasattr(slot, '_type'):
                slot_type = slot._type
            else:
                slot_type = type(slot)
            
            # if slot type is a string and expression has no string markers, add them
            if (slot_type in (str, 'string')) and not ('"' in expression or "'" in expression):
                expression = "'%s'" % expression
            
            try:
                value = eval(expression)
            except Exception:
                qWarning('Publisher.fill_message_slots(): failed to evaluate expression: %s' % (expression))
            else:
                setattr(message, slot_name, value)
        
        
    @Slot(int)
    def timeout(self, publisher_id):
        publisher_info = self.publishers_[publisher_id]
        publisher_info['counter'] += 1
        message = publisher_info['type']()
        self.fill_message_slots(message, publisher_info['topic_name'], publisher_info['expressions'], publisher_info['counter'])
        publisher_info['publisher'].publish(message)


    @Slot()
    def on_remove_publisher_button_clicked(self):
        selection = self.publishers_tree_widget.selectedIndexes()
        items = [self.publishers_tree_widget.itemFromIndex(index) for index in selection]
        uniqueItems = list(set(items))
        for item in uniqueItems:
            self.remove_publisher_item_(item)
    
            
    def remove_publisher_item_(self, item):
        self.remove_publisher(item.publisher_id)
        index = self.publishers_tree_widget.indexOfTopLevelItem(item)
        while item and index < 0:
            item = item.parent()
            index = self.publishers_tree_widget.indexOfTopLevelItem(item)
        if index >= 0:
            self.publishers_tree_widget.takeTopLevelItem(index)
        else:
            qDebug('Publisher.on_remove_publisher_button_clicked(): no top level item found.')


    @Slot('QPoint')
    def on_publishers_tree_widget_customContextMenuRequested(self, pos):
        item = self.publishers_tree_widget.itemAt(pos)
        if not item:
            return

        menu = QMenu(self)
        action_remove_publisher = menu.addAction("Remove Publisher")
        action = menu.exec_(self.publishers_tree_widget.mapToGlobal(pos))
        if action == action_remove_publisher:
            self.remove_publisher_item_(item)

    
    @Slot()
    def on_clear_button_clicked(self):
        self.clean_up_publishers()


    def save_settings(self, global_settings, perspective_settings):
        publisher_copies = []
        for publisher in self.publishers_.values():
            publisher_copy = {}
            publisher_copy.update(publisher)
            publisher_copy['enabled'] = False
            del publisher_copy['timer']
            del publisher_copy['type']
            del publisher_copy['publisher']
            del publisher_copy['widgetItem']
            publisher_copies.append(publisher_copy)
        perspective_settings.set_value('publishers', repr(publisher_copies))
        
        
    def restore_settings(self, global_settings, perspective_settings):
        publishers = eval(perspective_settings.value('publishers', '[]'))
        for publisher in publishers:
            self._add_publisher(publisher)
        
        
    def remove_publisher(self, publisher_id):
        publisher_info = self.publishers_[publisher_id]
        publisher_info['timer'].stop()
        publisher_info['publisher'].unregister()
        del self.publishers_[publisher_id]


    def clean_up_publishers(self):
        self.publishers_tree_widget.clear()
        for publisher_info in self.publishers_.values():
            publisher_info['timer'].stop()
            publisher_info['publisher'].unregister()
        self.publishers_ = {}


    def set_name(self, name):
        self.setWindowTitle(name) 


    # override Qt's closeEvent() method to trigger plugin unloading
    def closeEvent(self, event):
        QDockWidget.closeEvent(self, event)
        if event.isAccepted():
            self.deleteLater()


    def close_plugin(self):
        self.clean_up_publishers()
        QDockWidget.close(self)
