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

from rosgraph_msgs.msg import Log
import rospy

from python_qt_binding.QtCore import QObject

from .console_subscriber_dialog import ConsoleSubscriberDialog


class ConsoleSubscriber(QObject):
    """
    Subscribes to the /rosout_agg topic if a callback is provided and allows
    the user to change the currently subscribed topic.
    Also holds the message limit from ConsolesubscriberDialog
    """
    def __init__(self, callback=None):
        """
        :param callback: callback function to invoke when receiving a message ''function''
        """
        super(ConsoleSubscriber, self).__init__()
        self._msgcallback = callback
        self._currenttopic = "/rosout_agg"
        self._messagelimit = 20000
        if callback is not None:
            self._sub = rospy.Subscriber(self._currenttopic, Log, self._msgcallback)

    def show_dialog(self):
        dialog = ConsoleSubscriberDialog(rospy.get_published_topics(), self._messagelimit)
        for index in range(dialog.topic_combo.count()):
            if str(dialog.topic_combo.itemText(index)).find(self._currenttopic) != -1:
                dialog.topic_combo.setCurrentIndex(index)
        dialog.buffer_size_spin.setValue = self._messagelimit
        ok = dialog.exec_()
        ok = ok == 1
        if ok:
            temp = dialog.topic_combo.currentText()
            self.subscribe_topic(temp[:temp.find(' (')].strip())
            self._messagelimit = dialog.buffer_size_spin.value()
        return ok

    def unsubscribe_topic(self):
        if self._msgcallback is not None:
            self._sub.unregister()

    def subscribe_topic(self, topic):
        if self._msgcallback is not None:
            self.unsubscribe_topic()
            self._sub = rospy.Subscriber(topic, Log, self._msgcallback)
            self._currenttopic = topic

    def get_message_limit(self):
        return self._messagelimit

    def set_message_limit(self, limit):
        self._messagelimit = limit
