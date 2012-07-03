from message_list import Message
from message_list import MessageList
#filterlist holds a set of filters and will apply these filters to a
#dataset returning the reduced list for display
#It will have a widget when completed
from QtCore import qDebug, QModelIndex

class Filter:
    def __init__(self):
        self._enabled    = True     #enable/disable filter
        self._filtertext = ''       
        self._include    = True     
        self._regex      = False    
        
        # These apply locations will be unioned together
        self._applys = []           #list of which _messagemembers this filter applys to
        for member in Message()._messagemembers: 
            self._applys.append((member,True))
        self._applys = dict(self._applys)

class FilteredList(MessageList):
    def __init__(self):
        MessageList.__init__(self)
        self._filters =[]

        #an array of entries in _messagelist that conform to all _filters
        self._filteredlist = []
        self._deleteindexes = []

        #one filter for each attribute
        for member1 in Message()._messagemembers:
            applys = []
        #one dict entry for each attribute
            for member2 in Message()._messagemembers:
                applys.append((member2,member1 == member2))
            self.append_filter('',dict(applys))
    
    def test_conforms_to_filters(self, msgtext, severity, node, date, topic, location):
        return self.conforms_to_filters(Message(msgtext,severity,node,date,topic,location))
    
    def special_chars(self, text):
        if text.find('(') != -1 or text.find('AND') != -1 or text.find('OR') != -1:
            return True
        return False
    
    def bool_recurse(self, filter_, text, message):
        text= text.strip()
        if text.find('AND') == -1 and text.find('OR') == -1:
            for member in message._messagemembers:
                if filter_._applys[member] is True and getattr(message,member).find(str(text)) is -1:
                    return False
            return True
        else:
            if text.find('OR') != -1:
                left = text[:text.find('OR')].strip()
                right = text[text.find('OR')+2:].strip()
                return self.bool_recurse(filter_,left,message) or self.bool_recurse(filter_,right,message)
            elif text.find('AND') != -1:
                left = text[:text.find('AND')].strip()
                right = text[text.find('AND')+3:].strip()
                return self.bool_recurse(filter_,left,message) and self.bool_recurse(filter_,right,message)
            else:
                raise Exception('Malformed Boolean expression.')
            #do a recurse on just the bool bits

    def paren_recurse(self, filter_,text, message):
        if text is None:
            text=filter_._filtertext
        if text.find('(') == -1:
            return self.bool_recurse(filter_, text, message) 
        elif text.find('(') != -1:
            #shave off the parts
            left = text[:text.find('(')].strip()
            mid = text[text.find('(')+1:text.find(')')].strip()
            right = text[text.find(')')+1:].strip()

            left_op_or = None
            right_op_or = None

            #find the relevant "AND" or "OR" parts
            if left.strip() != '':
                if left.find('OR') != -1 and left.find('AND') == -1 :
                    left_op_or = True
                    left = left[:left.find('OR')]
                elif left.find('OR') == -1 and left.find('AND') != -1 : 
                    left_op_or = False
                    left = left[:left.find('AND')]
                elif left.rfind('OR') > left.rfind('AND'):
                    left_op_or = True
                    left = left[:left.rfind('OR')]
                elif left.rfind('OR') < left.rfind('AND'):
                    left_op_or = False
                    left = left[:left.rfind('AND'):]
                else:
                    raise Exception('Malformed Boolean expression.')

            if right.strip() != '':
                if right.find('OR') != -1 and right.find('AND') == -1 :
                    right_op_or = True
                    right = right[right.find('OR')+2:]
                elif right.find('OR') == -1 and right.find('AND') != -1 : 
                    right_op_or = False
                    right = right[right.find('AND')+3:]
                elif right.find('OR') < right.find('AND'):
                    right_op_or = True
                    right = right[right.find('OR')+2:]
                elif right.find('OR') > right.find('AND'):
                    right_op_or = False
                    right = right[right.find('AND')+3:]
                else:
                    raise Exception('Malformed Boolean expression.')
            
            if left_op_or is None and right_op_or is None:
                return self.paren_recurse(filter_,mid,message)
            elif left_op_or is None:
                if right_op_or:
                    return self.paren_recurse(filter_,mid,message) or self.paren_recurse(filter_,right,message)
                else:
                    return self.paren_recurse(filter_,mid,message) and self.paren_recurse(filter_,right,message)
            elif right_op_or is None:
                if left_op_or:
                    return self.paren_recurse(filter_,left,message) or self.paren_recurse(filter_,mid,message)
                else:
                    return self.paren_recurse(filter_,left,message) and self.paren_recurse(filter_,mid,message)
            else:
                if left_op_or:
                    if right_op_or:
                        return self.paren_recurse(filter_,left,message) or self.paren_recurse(filter_,mid,message) or self.paren_recurse(filter_,right,message)
                    else:
                        return self.paren_recurse(filter_,left,message) or self.paren_recurse(filter_,mid,message) and self.paren_recurse(filter_,right,message)
                else:
                    if right_op_or:
                        return self.paren_recurse(filter_,left,message) and self.paren_recurse(filter_,mid,message) or self.paren_recurse(filter_,right,message)
                    else:
                        return self.paren_recurse(filter_,left,message) and self.paren_recurse(filter_,mid,message) and self.paren_recurse(filter_,right,message)
                
        
    def match_filter(self,afilter,message):
        #TODO implement time range filtering
        #TODO implement regex filtering
        filtertext=afilter._filtertext
        if filtertext == '':
            return True
        try:
            if self.special_chars(filtertext):
                if filtertext.count('(') == filtertext.count(')'):
                    return self.paren_recurse(afilter, None, message)
                else:
                    raise Exception('Contains unmatched parentheses. ')
        except Exception as e:
            print afilter._filtertext + ': ' + str(e) + ' Processing as text filter.'

        for member in message._messagemembers:
            if afilter._applys[member] is True:
                value = getattr(message,member)
                if member == '_time':
                    mintime = filtertext[:filtertext.find(':')]
                    maxtime = filtertext[filtertext.find(':')+1:]

                    if value < float(mintime) or value > float(maxtime):
                        return False
                elif value.find(filtertext) is -1:
                   return False
        return True

    def conforms_to_filters(self, message):
        #handles iterating over each filter and returns False if any filter doesn't match its include value
        returnbool=True
        for afilter in self._filters:
            if afilter._enabled:
                if self.match_filter(afilter,message) is not afilter._include:
                    returnbool=False
            if returnbool is False:
                break
        return returnbool 

    def rebuild_filtered_list(self):
        #TODO consider smarter rebuild methods that don't have to trash the whole list
        self._filteredlist = []
        self._deleteindexes = []
        for index, message in enumerate(self._messagelist):
            if self.conforms_to_filters(message):
                self._filteredlist.append(message)
                self._deleteindexes.append(index)

    def append_filter(self, filtertext, filterdict):
        newfilter=Filter()
        newfilter._filtertext=filtertext
        newfilter._applys=filterdict
        self._filters.append(newfilter)
        self.rebuild_filtered_list()

    def delete_filter(self, index):
        if index < len(self._filters) and index>=0:
            del self._filters[index]
            self.rebuild_filtered_list()

    def move_filter_down(self, index):
        if index < len(self._filters)-1:
            temp=self._filters[index]
            self._filters[index]=self._filters[index+1]
            self._filters[index+1]=temp
            self.rebuild_filtered_list()

    def move_filter_up(self, index):
        if index > 0:
            temp=self._filters[index]
            self._filters[index]=self._filters[index-1]
            self._filters[index-1]=temp
            self.rebuild_filtered_list()
    
    def add_message(self, message, severity, node, time, topics, location):
        newmessage=Message(message,severity,node,time,topics,location)
        self._messagelist.append(newmessage)
        if self.conforms_to_filters(newmessage) is True:
            self._filteredlist.append(newmessage)
            self.manualsort()
            self.rebuild_filtered_list()

    def get_message_list(self):
        return self._filteredlist
    
    def alter_filter_text(self,index, text):
        if index >= 0 and index < len(self._filters):
            self._filters[index]._filtertext = text
            self.rebuild_filtered_list()

    def sort(self, col, order):
        MessageList.sort(self, col, order)
        self.rebuild_filtered_list()
    
    def remove_rows(self, datamodel, selection):
        if len(selection) is 0:
            if len(self.get_message_list())>0:
                datamodel.beginRemoveRows(QModelIndex(),0,len(self.get_message_list())-1)
                del self.get_message_list()[0:len(self.get_message_list())-1]
                del self._messagelist[0:len(self._messagelist)-1]
                datamodel.endRemoveRows()
                datamodel.beginRemoveRows(QModelIndex(),0,0) #NOTE Workaround for phantom table row bug
                del self.get_message_list()[0]
                del self._messagelist[0]
                datamodel.endRemoveRows()
        else:   
            rowlist = []
            for current in selection:
                rowlist.append(current.row())
            rowlist = list(set(rowlist))
            rowlist.sort(reverse=True)
            for row in rowlist:
                datamodel.beginRemoveRows(QModelIndex(),row,row)
                del self.get_message_list()[row]
                del self._messagelist[self._deleteindexes[row]]
                datamodel.endRemoveRows()
        self.rebuild_filtered_list()
        return True

    def message_members(self):
        return Message()._messagemembers

    def get_unique_col_data(self, index):
        uniques_list = set()
        for message in self._messagelist:
            uniques_list.add(getattr(message,Message()._messagemembers[index]))
        return list(uniques_list)
