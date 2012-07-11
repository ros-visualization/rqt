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

from message import Message
from message_list import MessageList
from QtGui import QBrush, QSortFilterProxyModel
from QtCore import QDateTime, Qt, QVariant, qWarning

from message_filter import MessageFilter

class MessageProxyModel(QSortFilterProxyModel):
    """
    Provides sorting and filtering capabilities for the MessageDataModel.
    Filtering is based on standard boolean operations. Base units in boolean
    operations are considered true if they exist in the string they are applied to.
    """
    def __init__(self):
        super(QSortFilterProxyModel, self).__init__()
        self._filters = []
        self.setDynamicSortFilter(True);

        # one filter for each attribute
        for member1 in MessageList().message_members():
            applys = []
        # one dict entry for each attribute
            for member2 in MessageList().message_members():
                applys.append((member2, member1 == member2))
            self.append_filter('', dict(applys))
        self._and = '&'
        self._or = '|'
        self._not = '^'
    
    def filterAcceptsRow(self, sourcerow, sourceparent):
        rowdata = []
        for index in range(self.sourceModel().columnCount()):
            rowdata.append(self.sourceModel().index(sourcerow,index,sourceparent).data())
        try:
            retval = self._conforms_to_filters(rowdata)
        except Exception, e:
            qWarning(str(e))
            retval = True
        return retval

    def _check_special_chars(self, text):
        """
        Returns true if text contains a '(' or the variables _and, _or, _not
        """
        if text.find('(') != -1 or text.find(self._and) != -1 or text.find(self._or) != -1 or text.find(self._not) != -1:
            return True
        return False
    
    def _bool_recurse(self, filter_, text, message):
        """
        recursively parses a boolean string ignoring any parenthesis (For proper behavior they
        should be stripped with _paren_recurse prior to using this function)
        """
        text = text.strip()
        or_pos = text.find(self._or)
        and_pos = text.find(self._and)
        not_pos = text.find(self._not)
        if and_pos == -1 and or_pos == -1:
            if not_pos != -1:
                # Recursively process not
                return not self._bool_recurse(filter_, text[not_pos+len(self._not):], message)
            # Calculate, if any do not match it is False
            for member in message._messagemembers:
                if filter_._applys[member] is True and getattr(message, member).find(str(text)) is -1:
                    return False
            return True
        else:
            # Recursively partition ands/ors
            if or_pos != -1:
                left = text[:or_pos].strip()
                right = text[or_pos + len(self._or):].strip()
                return self._bool_recurse(filter_, left, message) or self._bool_recurse(filter_, right, message)
            elif and_pos != -1:
                left = text[:and_pos].strip()
                right = text[and_pos + len(self._and):].strip()
                return self._bool_recurse(filter_, left, message) and self._bool_recurse(filter_, right, message)
            else:
                raise Exception(self.tr('Malformed Boolean expression.'))
    
    def _paren_not_wrapper(self, filter_, text, message, not_):
        if not_:
            return not self._paren_recurse(filter_, text, message)
        else:
            return self._paren_recurse(filter_, text, message)

    def _paren_recurse(self, filter_, text, message):
        """
        Splits the filter into left, mid and right where mid is the contents of
        the parenthesis. Properly links together recursive calls to _paren_recurse.
        """
        if text is None:
            text = filter_._filtertext
        if text.find('(') == -1:
            return self._bool_recurse(filter_, text, message)
        elif text.find('(') != -1:
            # Find a matching pair of paren and split the text based on the locations
            unmatchedparens = 0
            leftindex = -1
            rightindex = -1
            for index, char in enumerate(text):
                if char == ')':
                    unmatchedparens -= 1
                    if unmatchedparens == 0:
                        rightindex = index
                        break
                elif char == '(':
                    unmatchedparens += 1
                    if unmatchedparens == 1:
                        leftindex = index
            left = text[:leftindex].strip()
            mid = text[leftindex + 1:rightindex].strip()
            right = text[rightindex + 1:].strip()
            left_op_or = None
            right_op_or = None
            # Find the operator to the left of the paren if any
            not_before_paren = False
            if left.find(self._not) != -1:
                not_before_paren = True
                left = left[:left.find(self._not)]
            if left.strip() != '':
                or_pos = left.find(self._or)
                and_pos = left.find(self._and)
                if or_pos != -1 and and_pos == -1:
                    left_op_or = True
                    left = left[:or_pos]
                elif or_pos == -1 and and_pos != -1:
                    left_op_or = False
                    left = left[:and_pos]
                elif or_pos > and_pos:
                    left_op_or = True
                    left = left[:or_pos]
                elif or_pos < and_pos:
                    left_op_or = False
                    left = left[:and_pos]
                else:
                    raise Exception(self.tr('Malformed Boolean expression.'))
            # Find the operator to the right of the paren if any
            if right.strip() != '':
                or_pos = right.find(self._or)
                and_pos = right.find(self._and)
                if or_pos != -1 and and_pos == -1 :
                    right_op_or = True
                    right = right[or_pos + len(self._or):]
                elif or_pos == -1 and and_pos != -1 :
                    right_op_or = False
                    right = right[and_pos + len(self._and):]
                elif or_pos < and_pos:
                    right_op_or = True
                    right = right[or_pos + len(self._or):]
                elif or_pos > and_pos:
                    right_op_or = False
                    right = right[and_pos + len(self._and):]
                else:
                    raise Exception(self.tr('Malformed Boolean expression.'))
            # Based on Found operators combine and return recursive calls
            if left_op_or is None and right_op_or is None:
                return self._paren_recurse(filter_, mid, message)
            elif left_op_or is None:
                if right_op_or:
                    return self._paren_recurse(filter_, mid, message) or self._paren_recurse(filter_, right, message)
                else:
                    return self._paren_recurse(filter_, mid, message) and self._paren_recurse(filter_, right, message)
            elif right_op_or is None:
                if left_op_or:
                    return self._paren_recurse(filter_, left, message) or self._paren_not_wrapper(filter_, mid, message)
                else:
                    return self._paren_recurse(filter_, left, message) and self._paren_not_wrapper(filter_, mid, message)
            else:
                if left_op_or:
                    if right_op_or:
                        return self._paren_recurse(filter_, left, message) or self._paren_not_wrapper(filter_, mid, message, not_before_paren) or self._paren_recurse(filter_, right, message)
                    else:
                        return self._paren_recurse(filter_, left, message) or self._paren_not_wrapper(filter_, mid, message, not_before_paren) and self._paren_recurse(filter_, right, message)
                else:
                    if right_op_or:
                        return self._paren_recurse(filter_, left, message) and self._paren_not_wrapper(filter_, mid, message, not_before_paren) or self._paren_recurse(filter_, right, message)
                    else:
                        return self._paren_recurse(filter_, left, message) and self._paren_not_wrapper(filter_, mid, message, not_before_paren) and self._paren_recurse(filter_, right, message)

    def _match_filter(self, afilter, message):
        """
        If the message complies with afilter then this function returns True.
        Otherwise if it fails to match it returns False.
        It will throw a generic Exception if the boolean is malformed.
        """
        filtertext = afilter._filtertext
        if filtertext == '':
            return True
        if self._check_special_chars(filtertext):
            if filtertext.count('(') == filtertext.count(')'):
                return self._paren_recurse(afilter, None, message)
            else:
                raise Exception(self.tr('Boolean filter Contains unmatched parentheses.'))
        for member in message._messagemembers:
            if afilter._applys[member] is True:
                value = getattr(message, member)
                if member == '_time':
                    mintime, maxtime = filtertext.split(':')
                    timeval = self.sourceModel().timestring_to_timedata(value)
                    if float(timeval) < float(mintime) or float(timeval) > float(maxtime):
                        return False
                elif value.find(filtertext) is -1:
                    return False
        return True

    def _conforms_to_filters(self, rowdata):
        """
        Handles iterating over each filter and returns False if any filter doesn't
        match a Message created from rowdata.
        """
        message = Message()
        rowdata[3] = self.sourceModel().timestring_to_timedata(rowdata[3])
        message.load_from_array(rowdata)

        returnbool = True
        for afilter in self._filters:
            if afilter._enabled:
                if self._match_filter(afilter, message) is not afilter._include:
                    returnbool = False
            if returnbool is False:
                break
        return returnbool


    def append_filter(self, filtertext, filterdict):
        newfilter = MessageFilter()
        newfilter._filtertext = filtertext
        newfilter._applys = filterdict
        self._filters.append(newfilter)
        self.reset()

    def delete_filter(self, index):
        if index < len(self._filters) and index >= 0:
            del self._filters[index]
        self.reset()

    def set_filter(self, index, text):
        if index >= 0 and index < len(self._filters):
            self._filters[index]._filtertext = text
            if self.sourceModel().message_members()[index] == '_time' and text != '':
                left, right = text.split(':')
                text = self.sourceModel().timedata_to_timestring(left) + ':'
                text += self.sourceModel().timedata_to_timestring(right)
            self.sourceModel().set_header_text(index, text)
        self.reset()
    
    def get_filter(self, index):
        return self._filters[index]._filtertext

    def get_and(self):
        return self._and

    def get_or(self):
        return self._or

    def get_not(self):
        return self._not 
