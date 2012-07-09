import os
import rospy
import rosnode, rosservice
import math, random, time # used for the expression eval context

from QtGui import QWidget, QDialog, QListWidgetItem, QDialogButtonBox
from QtCore import qWarning, Qt, QTimer, Signal, Slot, QDateTime
from qt_gui.qt_binding_helper import loadUi
from rosgraph_msgs.msg import Log
from datetime import datetime

class ListDialog(QDialog):
    def __init__(self, windowtitle, text, boxlist, selected=''):
        super(QDialog, self).__init__()
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'list_input_dialog.ui')
        loadUi(ui_file, self)
        self.list_box.addItems(boxlist)
        for index, item in enumerate(boxlist):
            if selected.find(item) != -1: 
                self.list_box.item(index).setSelected(True)
        self.setWindowTitle(windowtitle)

    @staticmethod
    def show(titletext, labeltext, itemlist, selectedlist):
        dlg = ListDialog(titletext, labeltext, itemlist, selectedlist)
        ok = dlg.exec_()
        ok = (ok == 1)
        textlist = dlg.list_box.selectedItems()
        retlist = []
        for item in textlist:
            retlist.append(item.text())
        return (retlist, ok)

class TimeDialog(QDialog):
    ignore_button_clicked = Signal()
    def __init__(self):
        super(QDialog, self).__init__()
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'time_dialog.ui')
        loadUi(ui_file, self)
        def click_handler(button):
            if button == self.button_box.button(QDialogButtonBox.Ignore):
                self.ignore_button_clicked.emit()
                self.accept()
        self.button_box.clicked.connect(click_handler)
    
    def set_time(self, mintime=None, maxtime=None):
        if maxtime is None:
            time = datetime.now()
            self.min_dateedit.setDateTime(time)
            self.max_dateedit.setDateTime(time)
        else:
            dtime = QDateTime()
            dtime.setTime_t(int(mintime[:mintime.find('.')]))
            dtime = dtime.addMSecs(int(mintime[mintime.find('.')+1:mintime.find('.')+4]))
            self.min_dateedit.setDateTime(dtime)
            dtime = QDateTime()
            dtime.setTime_t(int(maxtime[:maxtime.find('.')]))
            dtime = dtime.addMSecs(int(maxtime[maxtime.find('.')+1:maxtime.find('.')+4]))
            self.max_dateedit.setDateTime(dtime)

class SetupDialog(QDialog):
    def __init__(self, context, callback):
        super(QDialog, self).__init__()
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'setup_dialog.ui')
        loadUi(ui_file, self)

        self._currenttopic = "/rosout_agg"
        self._topics = rospy.get_published_topics()
        self._topics.sort(reverse=True)
        for topic in self._topics:
            self.topic_combo.addItem(topic[0] + ' (' + topic[1] + ')')
        self._msgcallback = callback
        self._sub = rospy.Subscriber(self._currenttopic, Log, self._msgcallback)

        self._node_index = -1           # captures current list level
        self._logger_index = -1         # needed for the first calls since "currentItem" is not set 
        self._level_index = -1          # when programatically calling a callback from a callback

        self.node_list.currentRowChanged[int].connect(self.node_changed)
        self.logger_list.currentRowChanged[int].connect(self.logger_changed)
        self.level_list.currentRowChanged[int].connect(self.level_changed)
        self.topic_combo.activated[int].connect(self.topic_changed)

        self.refresh_button.clicked[bool].connect(self.refresh_nodes)
        self._current_loggers = []
        self._current_levels = {}

        # variables for use in fill_message_slots
        self._eval_locals = {}
        self._eval_locals.update(math.__dict__)
        self._eval_locals.update(random.__dict__)
        self._eval_locals.update(time.__dict__)
        del self._eval_locals['__name__']
        del self._eval_locals['__doc__']

    # function from service_caller to fill arbitrary messages with a list of slotname/key 
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
            qWarning('SetupDialog.fill_message_slots(): can not convert expression to slot type: %s -> %s' % (type(value), slot_type))
        else:
            qWarning('SetupDialog.fill_message_slots(): failed to evaluate expression: %s' % (expression))
        return None

    def unsub_topic(self):
        self._sub.unregister()

    def topic_changed(self):
        index = self.topic_combo.itemText(self.topic_combo.currentIndex()).find(' (')
        if index is not -1:
            self._currenttopic = self.topic_combo.itemText(self.topic_combo.currentIndex())[:index]
            self.unsub_topic()
            self._sub = rospy.Subscriber(self._currenttopic, Log, self._msgcallback)

    def get_levels(self):
        return ['Debug', 'Info', 'Warn', 'Error', 'Fatal']

    def get_loggers(self, node):
        self.refresh_loggers(node)
        return self._current_loggers

    def refresh_nodes(self):
        self.level_list.clear()
        self.logger_list.clear()
        self.node_list.clear()
        nodes = rosnode.get_node_names()
        for name in sorted(nodes):
            for service in rosservice.get_service_list(name):
                if service == name + '/set_logger_level':
                    self.node_list.addItem(name)

    def refresh_loggers(self, node):
        self._current_loggers = []
        self._current_levels = {}
        servicename = node + '/get_loggers'
        try:
            service = rosservice.get_service_class_by_name(servicename)
        except rosservice.ROSServiceIOException, e:
            qWarning(str(e))
            return []

        request = service._request_class()
        proxy = rospy.ServiceProxy(str(servicename), service)
        try:
            response = proxy(request)
        except rospy.ServiceException, e:
            qWarning('SetupDialog.get_loggers(): request:\n%s' % (type(request)))
            qWarning('SetupDialog.get_loggers() "%s":\n%s' % (servicename, e))
            return []

        if response._slot_types[0] == 'roscpp/Logger[]':
            for logger in getattr(response, response.__slots__[0]):
                self._current_loggers.append(getattr(logger, 'name'))
                self._current_levels[getattr(logger, 'name')] = getattr(logger, 'level')
        else:
            qWarning(repr(response)) 

    def node_changed(self, newrow):
        if newrow == -1:
            return
        if newrow < 0 or newrow >= self.node_list.count():
            qWarning('Node row %s out of bounds. Current count: %s' % (newrow,self.node_list.count()))
            return
        self._node_index = newrow
        self.logger_list.clear()
        self.level_list.clear()
        loggers = self.get_loggers(self.node_list.item(newrow).text())
        if len(loggers) is 0:
            return
        for logger in sorted(loggers):
            self.logger_list.addItem(logger)
        if self.logger_list.count() != 0:
            self.logger_list.setCurrentRow(0)

    def logger_changed(self, newrow):
        if newrow == -1:
            return
        if newrow < 0 or newrow >= self.logger_list.count():
            qWarning('Logger row %s out of bounds. Current count: %s' % (newrow,self.logger_list.count()))
            return
        self._logger_index = newrow
        
        if self.level_list.count() == 0:
            for level in self.get_levels():
                self.level_list.addItem(level)
        for index in range(self.level_list.count()):
            if self.level_list.item(index).text().lower() == self._current_levels[self.logger_list.item(newrow).text()].lower():
                self.level_list.setCurrentRow(index)

    def level_changed(self, newrow):
        if newrow == -1:
            return
        if newrow < 0 or newrow >= self.level_list.count():
            qWarning('Level row %s out of bounds. Current count: %s' % (newrow,self.level_list.count()))
            return
        self._level_index = newrow
        servicename = self.node_list.item(self._node_index).text() + '/set_logger_level'
        currentlogger = self.logger_list.item(self._logger_index).text()
        currentlevel = self.level_list.item(newrow).text()
        if self._current_levels[currentlogger].lower() == currentlevel.lower():
            return

        service = rosservice.get_service_class_by_name(servicename)
        request = service._request_class()
        self.fill_message_slots(request, servicename, {servicename + '/logger':currentlogger, servicename + '/level':currentlevel}, 0)
        proxy = rospy.ServiceProxy(str(servicename), service)
        try:
            response = proxy(request)
            self._current_levels[currentlogger] = currentlevel.upper()
        except rospy.ServiceException, e:
            qWarning('SetupDialog.level_changed(): request:\n%r' % (request))
            qWarning('SetupDialog.level_changed() "%s":\n%s' % (servicename, e))
