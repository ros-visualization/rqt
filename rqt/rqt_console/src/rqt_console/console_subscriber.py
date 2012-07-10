import rospy
import rosnode, rosservice

import qt_gui.qt_binding_helper  # @UnusedImport
from QtCore import QObject, qWarning
from qt_gui.qt_binding_helper import loadUi
from rosgraph_msgs.msg import Log

from console_subscriber_dialog import ConsoleSubscriberDialog

class ConsoleSubscriber(QObject):
    def __init__(self, callback=None):
        super(ConsoleSubscriber, self).__init__()
        if callback is None:
            callback = self._default_callback
        self._msgcallback = callback
        self._currenttopic = "/rosout_agg"
        self._sub = rospy.Subscriber(self._currenttopic, Log, self._msgcallback)
        
    def _default_callback(self):
        pass

    def show_dialog(self):
        dialog = ConsoleSubscriberDialog(self, rospy.get_published_topics())
        dialog.exec_()

    def unsubscribe_topic(self):
        self._sub.unregister()
    
    def subscribe_topic(self, topic):
        self.unsubscribe_topic()
        self._sub = rospy.Subscriber(topic, Log, self._msgcallback)

    def get_levels(self):
        return [self.tr('Debug'), self.tr('Info'), self.tr('Warn'), self.tr('Error'), self.tr('Fatal')]

    def get_loggers(self, node):
        if self._refresh_loggers(node):
            return self._current_loggers
        else:
            return []
    
    def get_node_names(self):
        set_logger_level_nodes = []
        nodes = rosnode.get_node_names()
        for name in sorted(nodes):
            for service in rosservice.get_service_list(name):
                if service == name + '/set_logger_level':
                    set_logger_level_nodes.append(name)
        return set_logger_level_nodes

    def _refresh_loggers(self, node):
        self._current_loggers = []
        self._current_levels = {}
        servicename = node + '/get_loggers'
        try:
            service = rosservice.get_service_class_by_name(servicename)
        except rosservice.ROSServiceIOException, e:
            qWarning(str(e))
            return False 
        request = service._request_class()
        proxy = rospy.ServiceProxy(str(servicename), service)
        try:
            response = proxy(request)
        except rospy.ServiceException, e:
            qWarning(self.tr('SetupDialog.get_loggers(): request:\n%s' % (type(request))))
            qWarning(self.tr('SetupDialog.get_loggers() "%s":\n%s' % (servicename, e)))
            return False

        if response._slot_types[0] == 'roscpp/Logger[]':
            for logger in getattr(response, response.__slots__[0]):
                self._current_loggers.append(getattr(logger, 'name'))
                self._current_levels[getattr(logger, 'name')] = getattr(logger, 'level')
        else:
            qWarning(repr(response))
            return False
        return True

    def send_logger_change_message(self, node, logger, level):
        servicename = node + '/set_logger_level'
        if self._current_levels[logger].lower() == level.lower():
            return

        service = rosservice.get_service_class_by_name(servicename)
        request = service._request_class()
        setattr(request, 'logger', logger)
        setattr(request, 'level', level)
        proxy = rospy.ServiceProxy(str(servicename), service)
        try:
            response = proxy(request)
            self._current_levels[logger] = level.upper()
        except rospy.ServiceException, e:
            qWarning(self.tr('SetupDialog.level_changed(): request:\n%r' % (request)))
            qWarning(self.tr('SetupDialog.level_changed() "%s":\n%s' % (servicename, e)))
