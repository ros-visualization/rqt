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

from ..message import Message


class FilterCollection:
    """
    This class provides an interface to filter Message objects based on
    of a set of filters which will be boolean combined with either 'or'
    or 'and' based on the combination type passed by the user.
    ''True'' for and combine and ''False'' for or combine
    """
    def __init__(self, proxymodel):
        """
        :param proxymodel: , ''QSortFilterProxyModel''
        """
        self._filters = []
        self._proxymodel = proxymodel

    def test_message_array(self, message):
        """
        test_message function for an array formatted message
        :param message: array of the message member data in order ''list'':
                        message text ''str'', severity ''str'', node ''str'',
                        time in seconds with decimals ''str'', topic ''str'',
        """
        newmessage = Message()
        message[3] = self._proxymodel.sourceModel().timestring_to_timedata(message[3])
        message = newmessage.load_from_array(message)
        return self.test_message(message)

    def test_message(self, message):
        """
        Tests if the message matches the entire list of filters.
        if passed an array of the 6 data elements of a message it will build one
        :param message: message to be tested against the filters, ''Message''
        :returns: True if the message matches the filters, ''bool''
        """
        for item in self._filters:
            if item.is_enabled() and item.test_message(message):
                return True
        return False

    def append(self, newfilter):
        """
        Adds a new filter to the filter list and returns the index
        :returns: The index of the filter appended, ''int''
        """
        self._filters.append(newfilter)

    def count_enabled_filters(self):
        enabled = 0
        for item in self._filters:
            if item.is_enabled():
                enabled += 1
        return enabled

    def __len__(self):
        return len(self._filters)

    def count(self):
        return len(self._filters)

    def __delitem__(self, index):
        del self._filters[index]
