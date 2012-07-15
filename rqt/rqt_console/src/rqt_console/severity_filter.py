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
from QtCore import QObject, Signal

from message import Message

class SeverityFilter(QObject):
    """
    Contains filter logic for a single filter
    """
    filter_changed_signal = Signal()
    def __init__(self):
        super(SeverityFilter, self).__init__()
        #self._widget = SeverityFilterWidget()
        self._debug = False
        self._info = False
        self._warning = False
        self._error = False
        self._fatal = False
        self._enabled = True

    def set_debug(self, checked):
        self._debug = checked
        if self._enabled:
            self.filter_changed_signal.emit()

    def set_info(self, checked):
        self._info = checked
        if self._enabled:
            self.filter_changed_signal.emit()

    def set_warning(self, checked):
        self._warning = checked
        if self._enabled:
            self.filter_changed_signal.emit()

    def set_error(self, checked):
        self._error = checked
        if self._enabled:
            self.filter_changed_signal.emit()

    def set_fatal(self, checked):
        self._fatal = checked
        if self._enabled:
            self.filter_changed_signal.emit()

    def set_enabled(self, checked):
        self._enabled = checked
        if self._enabled:
            self.filter_changed_signal.emit()

    def is_enabled(self):
        return self._enabled

    def message_test(self, message):
        """
        Tests if the message matches the filter.
        
        :param message: the message to be tested against the filters, ''Message''
        :returns: True if the message matches, ''bool''
        """
        if self._debug and message._severity.lower() == 'debug':
            return True
        if self._info and message._severity.lower() == 'info':
            return True
        if self._warning and message._severity.lower() == 'warning':
            return True
        if self._error and message._severity.lower() == 'error':
            return True
        if self._fatal and message._severity.lower() == 'fatal':
            return True
        return False

