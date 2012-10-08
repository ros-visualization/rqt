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

from .message import Message


class MessageList(object):
    """
    Partially simulates a two dimensional list with a single dimensional list of
    message objects. Also provides utility functions to provide data in useful formats
    """
    def __init__(self):
        self._messagelist = []

    def column_count(self):
        return len(Message.get_message_members())

    def get_message_list(self):
        return self._messagelist

    def message_members(self):
        return Message.get_message_members()

    def append_from_text(self, text):
        newmessage = Message()
        newmessage.file_load(text)
        self._messagelist.append(newmessage)

    def get_data(self, row, col):
        if row >= 0 and row < len(self.get_message_list()) and col >= 0 and col < len(Message.get_message_members()):
            return self.get_message_list()[row].get_data(col)
        else:
            raise IndexError

    def get_unique_col_data(self, index):
        """
        :param index: col index, ''int''
        :returns: a unique list of data index, ''list[str]''
        """
        uniques_list = set()
        for message in self._messagelist:
            uniques_list.add(getattr(message, self.message_members()[index]))
        return list(uniques_list)

    def add_message(self, msg):
        self._messagelist.append(Message(msg))

    def header_print(self):
        return Message.header_print()

    def get_messages_in_time_range(self, start_time, end_time = None):
        """
        :param start_time: time to start in timestamp form (including decimal
        fractions of a second is acceptable, ''unixtimestamp''
        :param end_time: time to end in timestamp form (including decimal
        fractions of a second is acceptable, ''unixtimestamp'' (Optional)
        :returns: list of messages in the time range ''list[message]''
        """
        message_list = self.get_message_list()
        time_range_list = []
        for message in message_list:
            msg_time = message.time_in_seconds()
            if float(msg_time) >= float(start_time) and (end_time is None or float(msg_time) <= float(end_time)):
                time_range_list.append(message)
        return time_range_list
