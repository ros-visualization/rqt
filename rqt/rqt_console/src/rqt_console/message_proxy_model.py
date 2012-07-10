from message import Message
from message_list import MessageList
from QtGui import QBrush, QSortFilterProxyModel
from QtCore import QDateTime, Qt, QVariant

class Filter:
    def __init__(self):
        self._enabled = True
        self._filtertext = ''
        self._include = True

        # These apply locations will be unioned together
        self._applys = []           #list of which _messagemembers this filter applys to
        for member in MessageList().message_members():
            self._applys.append((member, True))
        self._applys = dict(self._applys)

class MessageProxyModel(QSortFilterProxyModel):
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
        return self._conforms_to_filters(rowdata)

    def _check_special_chars(self, text):
        if text.find('(') != -1 or text.find(self._and) != -1 or text.find(self._or) != -1 or text.find(self._not) != -1:
            return True
        return False
    
    def _bool_recurse(self, filter_, text, message):
        text = text.strip()
        or_pos = text.find(self._or)
        and_pos = text.find(self._and)
        not_pos = text.find(self._not)
        if and_pos == -1 and or_pos == -1:
            if not_pos != -1:
                return not self._bool_recurse(filter_, text[not_pos+len(self._not):], message)
            for member in message._messagemembers:
                if filter_._applys[member] is True and getattr(message, member).find(str(text)) is -1:
                    return False
            return True
        else:
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
            #do a recurse on just the bool bits
    
    def _paren_not_wrapper(self, filter_, text, message, not_):
        if not_:
            return not self._paren_recurse(filter_, text, message)
        else:
            return self._paren_recurse(filter_, text, message)

    def _paren_recurse(self, filter_, text, message):
        if text is None:
            text = filter_._filtertext
        if text.find('(') == -1:
            return self._bool_recurse(filter_, text, message)
        elif text.find('(') != -1:
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
        #handles iterating over each filter and returns False if any filter doesn't match its include value
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
        newfilter = Filter()
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
