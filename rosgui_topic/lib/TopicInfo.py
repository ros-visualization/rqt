#!/usr/bin/env python

from __future__ import division, with_statement
import time
from StringIO import StringIO

import rosgui.QtBindingHelper #@UnusedImport
from QtCore import qDebug

import roslib
roslib.load_manifest('rosgui_topic')
import rospy
from rostopic import get_topic_class, ROSTopicHz

class TopicInfo(ROSTopicHz):

    def __init__(self, topic_name):
        super(TopicInfo, self).__init__(100)
        self.subscriber = None
        self.monitoring = False
        self._reset_data()
        try:
            self.message_class, self.topic_name, _ = get_topic_class(topic_name)
        except Exception:
            self.topic_name = None
            qDebug('TopicInfo.__init__(): can not get topic info for "%s"' % topic_name)
            return


    def _reset_data(self):
        self.last_message = None
        self.times = []
        self.timestamps = []
        self.sizes = []


    def toggle_monitoring(self):
        if self.monitoring:
            self.stop_monitoring()
        else:
            self.start_monitoring()


    def start_monitoring(self):
        self.monitoring = True
        if self.topic_name is not None:
            # FIXME: subscribing to class AnyMsg breaks other subscribers on same node
            self.subscriber = rospy.Subscriber(self.topic_name, self.message_class, self.message_callback)


    def stop_monitoring(self):
        self.monitoring = False
        self._reset_data()
        if self.subscriber is not None:
            self.subscriber.unregister()


    def message_callback(self, message):
        ROSTopicHz.callback_hz(self, message)
        with self.lock:
            self.timestamps.append(time.time())

            # FIXME: this only works for message of class AnyMsg
            #self.sizes.append(len(message._buff))
            # time consuming workaround...
            buff = StringIO()
            message.serialize(buff)
            self.sizes.append(buff.len)

            if len(self.timestamps) > self.window_size - 1:
                self.timestamps.pop(0)
                self.sizes.pop(0)
            assert(len(self.timestamps) == len(self.sizes))

            self.last_message = message


    def get_bw(self):
        if len(self.timestamps) < 2:
            return None, None, None, None
        with self.lock:
            total = sum(self.sizes)
            bytes_per_s = total / (time.time() - self.timestamps[0])
            mean_size = total / len(self.timestamps)
            max_size = max(self.sizes)
            min_size = min(self.sizes)
            return bytes_per_s, mean_size, min_size, max_size


    def get_hz(self):
        if not self.times:
            return None, None, None, None
        elif self.msg_tn == self.last_printed_tn:
            return 0#, 0, 0, 0
        with self.lock:
            n = len(self.times)
            mean = sum(self.times) / n
            rate = 1. / mean if mean > 0. else 0
            min_delta = min(self.times)
            max_delta = max(self.times)
        return rate, mean, min_delta, max_delta
