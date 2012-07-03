import os
import rospy
import rosnode,rosservice
import math, random, time # used for the expression eval context

from QtGui import QWidget, QDialog, QListWidgetItem, QDialogButtonBox
from QtCore import qDebug, Qt, QTimer, Signal, Slot
from qt_gui.qt_binding_helper import loadUi
from rosgraph_msgs.msg import Log 
from PyQt4 import QtCore
from datetime import datetime

class TimeDialog(QDialog):
    ignore_button_clicked = Signal()
    def __init__(self):
        super(QDialog, self).__init__()
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'time_dialog.ui')
        loadUi(ui_file,self)
        def click_handler(button):
            if button == self.button_box.button(QDialogButtonBox.Ignore):
                self.ignore_button_clicked.emit()
                self.accept()
        self.button_box.clicked.connect(click_handler)
        self.min_dateedit.setDateTime(datetime.now())
        self.max_dateedit.setDateTime(datetime.now())

class MainWindow(QWidget):
    def keyPressEvent(self,e):
        pass
#        print 'KeyPress' + str(e.key())
#        if e.key() == Qt.Key_Escape:
#            parent(self).close()

class SetupDialog(QDialog):
    def __init__(self,context, callback):
        super(QDialog, self).__init__()
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'setup_dialog.ui')
        loadUi(ui_file,self)

        self._currenttopic="/rosout_agg"
        self._topics=rospy.get_published_topics()
        self._topics.sort(reverse=True)
        for topic in self._topics:
            self.topic_combo.addItem(topic[0]+' ('+topic[1]+')')
        self._msgcallback=callback
        self._sub=rospy.Subscriber(self._currenttopic, Log, self._msgcallback)
        
#        self.refresh_nodes()
        self._first_call=True  
        
        self._node_index   = -1
        self._logger_index = -1
        self._level_index  = -1

        self.node_list.currentRowChanged[int].connect(self.node_changed)
        self.logger_list.currentRowChanged[int].connect(self.logger_changed)
        self.level_list.currentRowChanged[int].connect(self.level_changed)
        self.topic_combo.activated[int].connect(self.topic_changed)
        
        self.refresh_button.clicked[bool].connect(self.refresh_nodes)
        self._current_loggers=[]
        self._current_levels={}

# Local variables for use in fill_message_slots
        self._eval_locals = {}
        self._eval_locals.update(math.__dict__)
        self._eval_locals.update(random.__dict__)
        self._eval_locals.update(time.__dict__)
        del self._eval_locals['__name__']
        del self._eval_locals['__doc__']
        print 'init complete'
    
# function from service_caller to fill arbitrary messages with a list of slotname/key combos
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
        index=self.topic_combo.itemText(self.topic_combo.currentIndex()).find(' (')
        if index is not -1:
            self._currenttopic=self.topic_combo.itemText(self.topic_combo.currentIndex())[:index]
            print 'Subscribing to: ' + self._currenttopic
            self.unsub_topic()
            self._sub=rospy.Subscriber(self._currenttopic, Log, self._msgcallback)

    def refresh_nodes(self):
        print 'calling: refresh_nodes'
        self.node_list.clear()
        nodes = rosnode.get_node_names()
        for name in sorted(nodes):
            for service in rosservice.get_service_list(name):
                if service == name + '/set_logger_level':
                    self.node_list.addItem(name)

    def get_loggers(self,node):
        self.refresh_loggers(node)
        return self._current_loggers

    def refresh_loggers(self,node):
        print 'calling: refresh_loggers'
        self._current_loggers=[]
        self._current_levels={}
        servicename= node+'/get_loggers'
        service = rosservice.get_service_class_by_name(servicename)
        
        request = service._request_class()
        #self.fill_message_slots(request, servicename, {}, 0)
        proxy = rospy.ServiceProxy(str(servicename), service)
        try:
            response = proxy(request)
        except rospy.ServiceException, e:
            qDebug('SetupDialog.get_loggers(): request:\n%s' % (type(request)))
            qDebug('SetupDialog.get_loggers() "%s":\n%s' % (servicename, e))
            return []
            
        if response._slot_types[0] == 'roscpp/Logger[]':
            for logger in getattr(response,response.__slots__[0]):
                self._current_loggers.append(getattr(logger,'name'))
                self._current_levels[getattr(logger,'name')]=getattr(logger,'level')
        else:
            print repr(response) #got a strange response

    def get_levels(self):
        return ['Debug','Info','Warn','Error','Fatal']

    def node_changed(self,newrow):
        print 'calling: node_changed: ' + str(newrow)
        if self._first_call:
            self._first_call=False
            return
        self._node_index   = newrow
        
        if self.node_list.count() is 0:
            return
        self.logger_list.clear()
        self.level_list.clear()
        loggers = self.get_loggers(self.node_list.item(newrow).text())
        if len(loggers) is 0:
            return
        for logger in sorted(loggers):
            self.logger_list.addItem(logger)
        if self.logger_list.count() != 0:
            self.logger_list.setCurrentRow(0)

    def logger_changed(self,newrow):
        print 'calling: logger_changed: ' + str(newrow)
        self._logger_index = newrow
        
        if self.level_list.count() == 0:
            for level in self.get_levels():
                self.level_list.addItem(level)
        for index in range(self.level_list.count()):
            if self.level_list.item(index).text().lower() == self._current_levels[self.logger_list.item(newrow).text()].lower():
                self.level_list.setCurrentRow(index)
        print self._current_levels

    def level_changed(self,newrow):
        print 'calling: level_changed: ' + str(newrow)
        
        if self.logger_list.count() is 0 or self.level_list.count() is 0:
            return
        self._level_index = newrow
        servicename= self.node_list.item(self._node_index).text()+'/set_logger_level'
        currentlogger = self.logger_list.item(self._logger_index).text()
        currentlevel = self.level_list.item(newrow).text()
        if self._current_levels[currentlogger].lower() == currentlevel.lower():
            return

        service = rosservice.get_service_class_by_name(servicename)
        print servicename +':'+ currentlogger+':'+currentlevel

        request = service._request_class()
        self.fill_message_slots(request, servicename,{servicename+'/logger':currentlogger,servicename+'/level':currentlevel}, 0)
        proxy = rospy.ServiceProxy(str(servicename), service)
        try:
            response = proxy(request)
            self._current_levels[currentlogger]=currentlevel.upper()
        except rospy.ServiceException, e:
            qDebug('SetupDialog.level_changed(): request:\n%r' % (request))
            qDebug('SetupDialog.level_changed() "%s":\n%s' % (servicename, e))
        print self._current_levels
