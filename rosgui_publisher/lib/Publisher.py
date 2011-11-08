#!/usr/bin/env python

from __future__ import division
import inspect, os, sys
import math, random, time # used for the expression eval context

from rosgui.QtBindingHelper import loadUi
from QtCore import Qt, QTimer, QSignalMapper, Slot, qDebug, qWarning
from QtGui import QDockWidget, QTreeWidgetItem, QMenu

import roslib
roslib.load_manifest('rosgui_publisher')
import rosmsg, rospy
from roslib.msgs import load_package, REGISTERED_TYPES
from ExtendedComboBox import ExtendedComboBox

# main class inherits from the ui window class
class Publisher(QDockWidget):
    _column_names = ['topic', 'type', 'rate', 'enabled', 'expression']


    def __init__(self, parent, plugin_context):
        super(Publisher, self).__init__(plugin_context.main_window())
        self.setObjectName('Publisher')

        # create context for the expression eval statement
        self._eval_locals = {}
        self._eval_locals.update(math.__dict__)
        self._eval_locals.update(random.__dict__)
        self._eval_locals.update(time.__dict__)
        del self._eval_locals['__name__']
        del self._eval_locals['__doc__']

        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'Publisher.ui')
        loadUi(ui_file, self, {'ExtendedComboBox': ExtendedComboBox})

        if plugin_context.serial_number() > 1:
            self.setWindowTitle(self.windowTitle() + (' (%d)' % plugin_context.serial_number()))

        self._column_index = {}
        for column_name in self._column_names:
            self._column_index[column_name] = len(self._column_index)

        self._publishers = {}
        self._id_counter = 0
        for message_package_name in rosmsg.iterate_packages('.msg'):
            load_package(message_package_name)

        self._timeout_mapper = QSignalMapper(self)
        self._timeout_mapper.mapped[int].connect(self.timeout)

        self.publishers_tree_widget.itemChanged.connect(self.publishers_tree_widget_itemChanged)
        self.update_type_combo_box()

        # add our self to the main window
        plugin_context.main_window().addDockWidget(Qt.RightDockWidgetArea, self)


    def import_message_package(self, package_name):
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
            qDebug('Publisher.add_message_package(): failed to import %s' % (package_name + '.msg'))
            return
        message_types = inspect.getmembers(package_module, inspect.isclass)
        if len(message_types) == 0:
            qDebug('Publisher.add_message_package(%s): no message classes found in %s' % (package_name, package_module))
        for type_name, message_type in message_types:
            self._message_classes['%s/%s' % (package_name, type_name)] = message_type


    def update_type_combo_box(self):
        self.type_combo_box.clear()
        self.type_combo_box.addItems(sorted(REGISTERED_TYPES.keys()))


    @Slot()
    def on_add_publisher_button_clicked(self, publisher_info=None):
        # create publisher info
        if publisher_info is None:
            publisher_info = {}
        publisher_info['topic_name'] = str(self.topic_combo_box.currentText())
        publisher_info['type_name'] = str(self.type_combo_box.currentText())
        publisher_info['rate'] = float(self.frequency_combo_box.currentText())
        publisher_info['enabled'] = True
        self._add_publisher(publisher_info)


    def _add_publisher(self, publisher_info):
        publisher_info['publisher_id'] = self._id_counter
        self._id_counter += 1
        publisher_info['counter'] = 0
        publisher_info['enabled'] = publisher_info.get('enabled', False)
        publisher_info['expressions'] = publisher_info.get('expressions', {})

        publisher_info['message_instance'] = self._create_message_instance(publisher_info['type_name'])
        if publisher_info['message_instance'] is None:
            return

        # create publisher and timer
        publisher_info['publisher'] = rospy.Publisher(publisher_info['topic_name'], type(publisher_info['message_instance']))
        publisher_info['timer'] = QTimer(self)

        # add publisher info to _publishers dict and create signal mapping  
        self._publishers[publisher_info['publisher_id']] = publisher_info
        self._timeout_mapper.setMapping(publisher_info['timer'], publisher_info['publisher_id'])
        publisher_info['timer'].timeout.connect(self._timeout_mapper.map)
        if publisher_info['enabled']:
            publisher_info['timer'].start(int(1000.0 / publisher_info['rate']))

        self._show_publisher_in_tree_view(publisher_info)


    def _show_publisher_in_tree_view(self, publisher_info):
        # recursively create widget items for the message's slots 
        top_level_item = self._recursive_create_widget_items(None, publisher_info['topic_name'], publisher_info['message_instance']._type, publisher_info['message_instance'], publisher_info['publisher_id'], publisher_info['expressions'])

        # fill tree widget columns of top level item
        top_level_item.setText(self._column_index['enabled'], str(publisher_info['enabled']))
        top_level_item.setText(self._column_index['rate'], str(publisher_info['rate']))

        # add/replace top level item in tree widget
        for index in range(self.publishers_tree_widget.topLevelItemCount()):
            if publisher_info['publisher_id'] == self.publishers_tree_widget.topLevelItem(index).publisher_id:
                # publisher already found, replace item
                self.publishers_tree_widget.takeTopLevelItem(index)
                self.publishers_tree_widget.insertTopLevelItem(index, top_level_item)
                break
        else:
            self.publishers_tree_widget.addTopLevelItem(top_level_item)

        # resize columns
        self.publishers_tree_widget.expandAll()
        for i in range(self.publishers_tree_widget.columnCount()):
            self.publishers_tree_widget.resizeColumnToContents(i)


    def _recursive_create_widget_items(self, parent, topic_name, type_name, message, publisher_id, expressions):
        if parent is None:
            # show full topic name with preceding namespace on toplevel item
            topic_text = topic_name
        else:
            topic_text = topic_name.split('/')[-1]
        item = QTreeWidgetItem(parent)
        item.setText(self._column_index['topic'], topic_text)
        item.setText(self._column_index['type'], type_name)
        item.setFlags(item.flags() | Qt.ItemIsEditable)
        item.setData(0, Qt.UserRole, topic_name)
        item.publisher_id = publisher_id
        if hasattr(message, '__slots__') and hasattr(message, '_slot_types'):
            for slot_name, type_name in zip(message.__slots__, message._slot_types):
                self._recursive_create_widget_items(item, topic_name + '/' + slot_name, type_name, getattr(message, slot_name), publisher_id, expressions)

        elif type(message) in (list, tuple) and (len(message) > 0) and hasattr(message[0], '__slots__'):
            type_name = type_name.split('[', 1)[0]
            for index, slot in enumerate(message):
                self._recursive_create_widget_items(item, topic_name + '[%d]' % index, type_name, slot, publisher_id, expressions)

        else:
            if topic_name in expressions:
                item.setText(self._column_index['expression'], expressions[topic_name])
            elif not hasattr(message, '__slots__'):
                item.setText(self._column_index['expression'], repr(message))

        return item


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


    def _create_message_instance(self, type_str):
        base_type_str, array_size = self._extract_array_info(type_str)

        if roslib.genpy.is_simple(base_type_str):
            message = eval(roslib.genpy.default_value(type_str, ''))
        else:
            base_message_type = roslib.message.get_message_class(base_type_str)

            if array_size is not None:
                message = []
                for _ in range(array_size):
                    message.append(base_message_type())
            else:
                message = base_message_type()

        return message


    @Slot('QTreeWidgetItem*', int)
    def publishers_tree_widget_itemChanged(self, item, column):
        column_name = self._column_names[column]
        new_value = str(item.text(column))
        #qDebug('Publisher.on_treePublishers_itemChanged(): %s : %s' % (column_name, new_value))
        if not hasattr(item, 'publisher_id'):
            qDebug('Publisher.on_treePublishers_itemChanged(): no publisher_id found in: %s' % (item))
        else:
            publisher_info = self._publishers[item.publisher_id]

            if column_name == 'enabled':
                publisher_info['enabled'] = (new_value and new_value.lower() in ['1', 'true', 'yes'])
                item.setText(column, '%s' % publisher_info['enabled'])
                #qDebug('Publisher.on_treePublishers_itemChanged(): %s enabled: %s' % (publisher_info['topic_name'], publisher_info['enabled']))
                if publisher_info['enabled']:
                    publisher_info['timer'].start(int(1000.0 / publisher_info['rate']))
                else:
                    publisher_info['timer'].stop()

            elif column_name == 'type':
                type_name = new_value
                # create new slot
                slot_value = self._create_message_instance(type_name)

                # find parent slot
                topic_name = str(item.data(0, Qt.UserRole))
                slot_path = topic_name[len(publisher_info['topic_name']):].strip('/').split('/')
                parent_slot = eval('.'.join(["publisher_info['message_instance']"] + slot_path[:-1]))

                # find old slot
                slot_name = slot_path[-1]
                slot_index = parent_slot.__slots__.index(slot_name)

                # restore type if user value was invalid
                if slot_value is None:
                    qDebug('Publisher.on_treePublishers_itemChanged(): could not find type: %s' % (type_name))
                    item.setText(column, parent_slot._slot_types[slot_index])
                    return

                # replace old slot
                parent_slot._slot_types[slot_index] = type_name
                setattr(parent_slot, slot_name, slot_value)

                self._show_publisher_in_tree_view(publisher_info)

            elif column_name == 'rate':
                try:
                    rate = float(new_value)
                except Exception:
                    qDebug('Publisher.on_treePublishers_itemChanged(): could not parse rate value: %s' % (new_value))
                else:
                    if rate <= 0:
                        qDebug('Publisher.on_treePublishers_itemChanged(): invalid rate: %f' % (rate))
                    else:
                        publisher_info['rate'] = rate
                        qDebug('Publisher.on_treePublishers_itemChanged(): %s rate changed: %s' % (publisher_info['topic_name'], publisher_info['rate']))
                        if publisher_info['enabled']:
                            publisher_info['timer'].stop()
                            publisher_info['timer'].start(int(1000.0 / publisher_info['rate']))
                # make sure the column value reflects the actual rate
                item.setText(column, '%.2f' % publisher_info['rate'])

            elif column_name == 'expression':
                topic_name = str(item.data(0, Qt.UserRole))
                publisher_info['expressions'][topic_name] = new_value
                qDebug('Publisher.on_treePublishers_itemChanged(): %s expression: %s' % (topic_name, new_value))


    def fill_message_slots(self, message, topic_name, expressions, counter):
        if not hasattr(message, '__slots__'):
            return
        for slot_name in message.__slots__:
            slot_key = topic_name + '/' + slot_name

            # if no expression exists for this slot_key, continue with it's child slots
            if slot_key not in expressions:
                self.fill_message_slots(getattr(message, slot_name), slot_key, expressions, counter)
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

            self._eval_locals['i'] = counter
            value = self._evaluate_expression(expression, slot_type)
            if value is not None:
                setattr(message, slot_name, value)


    def _evaluate_expression(self, expression, slot_type):
        successful_eval = True
        successful_conversion = True

        try:
            # try to evaluate expression
            value = eval(expression, {}, self._eval_locals)
        except Exception:
            # just use expression-string as value
            value = expression
            successful_eval = False

        try:
            # try to convert value to right type
            value = slot_type(value)
        except Exception:
            successful_conversion = False

        if successful_conversion:
            return value
        elif successful_eval:
            qWarning('Publisher.fill_message_slots(): can not convert expression to slot type: %s -> %s' % (type(value), slot_type))
        else:
            qWarning('Publisher.fill_message_slots(): failed to evaluate expression: %s' % (expression))

        return None


    @Slot(int)
    def timeout(self, publisher_id):
        publisher_info = self._publishers[publisher_id]
        publisher_info['counter'] += 1
        self.fill_message_slots(publisher_info['message_instance'], publisher_info['topic_name'], publisher_info['expressions'], publisher_info['counter'])
        publisher_info['publisher'].publish(publisher_info['message_instance'])


    @Slot()
    def on_remove_publisher_button_clicked(self):
        selection = self.publishers_tree_widget.selectedIndexes()
        items = [self.publishers_tree_widget.itemFromIndex(index) for index in selection]
        uniqueItems = list(set(items))
        for item in uniqueItems:
            self._remove_publisher_item(item)


    def _remove_publisher_item(self, item):
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
        if action is action_remove_publisher:
            self._remove_publisher_item(item)


    @Slot()
    def on_clear_button_clicked(self):
        self.clean_up_publishers()


    def save_settings(self, global_settings, perspective_settings):
        publisher_copies = []
        for publisher in self._publishers.values():
            publisher_copy = {}
            publisher_copy.update(publisher)
            publisher_copy['enabled'] = False
            del publisher_copy['timer']
            del publisher_copy['message_instance']
            del publisher_copy['publisher']
            publisher_copies.append(publisher_copy)
        perspective_settings.set_value('publishers', repr(publisher_copies))


    def restore_settings(self, global_settings, perspective_settings):
        publishers = eval(perspective_settings.value('publishers', '[]'))
        for publisher in publishers:
            self._add_publisher(publisher)


    def remove_publisher(self, publisher_id):
        publisher_info = self._publishers[publisher_id]
        publisher_info['timer'].stop()
        publisher_info['publisher'].unregister()
        del self._publishers[publisher_id]


    def clean_up_publishers(self):
        self.publishers_tree_widget.clear()
        for publisher_info in self._publishers.values():
            publisher_info['timer'].stop()
            publisher_info['publisher'].unregister()
        self._publishers = {}


    def set_name(self, name):
        self.setWindowTitle(name)


    # override Qt's closeEvent() method to trigger plugin unloading
    def closeEvent(self, event):
        event.ignore()
        self.deleteLater()


    def close_plugin(self):
        self.clean_up_publishers()
        QDockWidget.close(self)
