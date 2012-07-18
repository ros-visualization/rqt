# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rosnode
import rospy
import rosservice

from qt_gui.qt_binding_helper import loadUi
from QtCore import QObject, qWarning


class LoggerLevelServiceCaller(QObject):
    """
    Handles service calls for getting lists of nodes and loggers
    Also handles sending change logger level changed messages
    """
    def __init__(self):
        super(LoggerLevelServiceCaller, self).__init__()

    def get_levels(self):
        return [self.tr('Debug'), self.tr('Info'), self.tr('Warn'), self.tr('Error'), self.tr('Fatal')]

    def get_loggers(self, node):
        if self._refresh_loggers(node):
            return self._current_loggers
        else:
            return []

    def get_node_names(self):
        """
        Gets a list of available services via a ros service call.
        Returns a list of all nodes that provide set_logger_level.
        """
        set_logger_level_nodes = []
        nodes = rosnode.get_node_names()
        for name in sorted(nodes):
            for service in rosservice.get_service_list(name):
                if service == name + '/set_logger_level':
                    set_logger_level_nodes.append(name)
        return set_logger_level_nodes

    def _refresh_loggers(self, node):
        """
        Gets and stores a list of loggers available for 'node'
        """
        self._current_loggers = []
        self._current_levels = {}
        servicename = node + '/get_loggers'
        try:
            service = rosservice.get_service_class_by_name(servicename)
        except rosservice.ROSServiceIOException as e:
            qWarning(e)
            return False
        request = service._request_class()
        proxy = rospy.ServiceProxy(str(servicename), service)
        try:
            response = proxy(request)
        except rospy.ServiceException as e:
            qWarning('SetupDialog.get_loggers(): request:\n%s' % (type(request)))
            qWarning('SetupDialog.get_loggers() "%s":\n%s' % (servicename, e))
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
        """
        Sends a logger level change request to 'node'.
        Returns True if the response is valid.
        Returns False if the request raises an exception
        Returns False if the request would not change the state
        """
        servicename = node + '/set_logger_level'
        if self._current_levels[logger].lower() == level.lower():
            return False

        service = rosservice.get_service_class_by_name(servicename)
        request = service._request_class()
        setattr(request, 'logger', logger)
        setattr(request, 'level', level)
        proxy = rospy.ServiceProxy(str(servicename), service)
        try:
            response = proxy(request)
            self._current_levels[logger] = level.upper()
        except rospy.ServiceException as e:
            qWarning('SetupDialog.level_changed(): request:\n%r' % (request))
            qWarning('SetupDialog.level_changed() "%s":\n%s' % (servicename, e))
            return False
        return True
