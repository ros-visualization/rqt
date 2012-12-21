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

from .base_filter import BaseFilter
from .message_filter import MessageFilter
from .node_filter import NodeFilter
from .severity_filter import SeverityFilter
from .topic_filter import TopicFilter


class CustomFilter(BaseFilter):
    """
    Contains filter logic for the custom filter which allows message, severity,
    node and topic filtering simultaniously. All of these filters must match
    together or the custom filter does not match
    """

    def __init__(self):
        super(CustomFilter, self).__init__()

        self._message = MessageFilter()
        self._message.filter_changed_signal.connect(self.relay_emit_signal)
        self._severity = SeverityFilter()
        self._severity.filter_changed_signal.connect(self.relay_emit_signal)
        self._node = NodeFilter()
        self._node.filter_changed_signal.connect(self.relay_emit_signal)
        self._topic = TopicFilter()
        self._topic.filter_changed_signal.connect(self.relay_emit_signal)

    def set_enabled(self, checked):
        """
        :signal: emits filter_changed_signal
        :param checked: enables the filters if checked is True''bool''
        """
        self._message.set_enabled(checked)
        self._severity.set_enabled(checked)
        self._node.set_enabled(checked)
        self._topic.set_enabled(checked)
        super(CustomFilter, self).set_enabled(checked)

    def relay_emit_signal(self):
        """
        Passes any signals emitted by the child filters along
        """
        self.start_emit_timer(1)

    def test_message(self, message):
        """
        Tests if the message matches the filter.
        :param message: the message to be tested against the filters, ''Message''
        :returns: True if the message matches all child filters, ''bool''
        """
        return self._message.test_message(message) and self._severity.test_message(message) and self._node.test_message(message) and self._topic.test_message(message)
