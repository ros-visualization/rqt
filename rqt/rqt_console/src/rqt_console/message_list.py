from QtCore import QModelIndex, Qt, qWarning
from rosgraph_msgs.msg import Log 

class Message(object):
    def __init__(self, message=None, severity=None, node=None, time=None, topics=None, location=None):
        self._message = message
        self._severity = severity
        self._node = node
        self._time = time
        self._topics = topics
        self._location = location
        self._messagemembers = ('_message', '_severity', '_node', '_time', '_topics', '_location')
    
    def message_members(self):
        return self._messagemembers

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

    def addMessage(self, message, severity, node, time, topics, location):
        self._messagelist.append(Message(message, severity, node, time, topics, location))

    def columnCount(self):
        return 6

    def get_message_list(self):
        return self._messagelist

    def message_members(self):
        return Message()._messagemembers

    def append_from_text(self, text):
        newmessage = Message()
        newmessage.file_load(text)
        self._messagelist.append(newmessage)

    def get_data(self, row, col):
        if row >= 0 and row < len(self.get_message_list()) and col >= 0 and col < 6:
            message = self.get_message_list()[row]
            return getattr(message,Message()._messagemembers[col])
        else:
            raise IndexError

    def get_unique_col_data(self, index):
        uniques_list = set()
        for message in self._messagelist:
            uniques_list.add(getattr(message, self.message_members()[index]))
        return list(uniques_list)

    def add_message(self, message, severity, node, time, topics, location):
        newmessage = Message(message, severity, node, time, topics, location)
        self._messagelist.append(newmessage)

