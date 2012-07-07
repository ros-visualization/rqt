from QtCore import Qt
from rosgraph_msgs.msg import Log #TODO See if we can't make it an array of type Log instead of making an object for it 

class Message(object):
    def __init__(self, message=None, severity=None, node=None, time=None, topics=None, location=None):
        self._message = message
        self._severity = severity
        self._node = node
        self._time = time
        self._topics = topics
        self._location = location
        self._messagemembers = ('_message', '_severity', '_node', '_time', '_topics', '_location')

    def CountElements(self):
        return 6 #returns the number of data elements contained in this object
    
    def file_print(self):
        text = self._node + ';'
        text += self._time + ';'
        text += self._severity + ';'
        text += self._topics + ';'
        text += self._location + ';'
        altered_message = self._message.replace('"','\\"')
        text += '"' + altered_message + '"\n'
        return text

    def file_load(self, text):
        sc_index = text.find(';') 
        if sc_index == -1:
            raise
        self._node = text[:sc_index]
        text = text[text.find(';')+1:]
        sc_index = text.find(';') 
        if sc_index == -1:
            raise
        self._time = text[:sc_index]
        text = text[text.find(';')+1:]
        sc_index = text.find(';') 
        if sc_index == -1:
            raise
        self._severity = text[:sc_index]
        text = text[text.find(';')+1:]
        sc_index = text.find(';') 
        if sc_index == -1:
            raise
        self._topics = text[:sc_index]
        text = text[text.find(';')+1:]
        sc_index = text.find(';') 
        if sc_index == -1:
            raise 
        self._location = text[:sc_index]
        text = text[sc_index+1:]
        text = text.replace('\\"','"')
        self._message = text[1:-2]
        return

    def pretty_print(self):
        text = 'Node: ' + self._node + '\n'
        text += 'Time: ' + self._time + '\n'
        text += 'Severity: ' + self._severity + '\n'
        text += 'Published Topics: ' + self._topics + '\n'
        text += '\n' + self._message + '\n'
        return text

class MessageList(object):
    def __init__(self):
        self._messagelist = []

        #default to sorting decending by time
        self._sortcol = 3
        self._sortdec = True

    def addMessage(self, message, severity, node, time, topics, location):
        self._messagelist.append(Message(message, severity, node, time, topics, location))

    def columnCount(self):
        return 6

    def getMessageList(self):
        return self._messagelist

    def sort(self, col, order):
        self._sortcol = col
        rev = False
        if order == Qt.DescendingOrder:
            rev = True
        self._sortdec = rev
        member = Message()._messagemembers[col]
        self._messagelist = sorted(self._messagelist, key=lambda message: getattr(message, member).lower(), reverse=rev)

