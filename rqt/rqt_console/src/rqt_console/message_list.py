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

    def manualsort(self):
        self.sort(self._sortcol, self._sortdec)

    def sort(self, col, order):
        self._sortcol = col
        rev = False
        if order == Qt.DescendingOrder:
            rev = True
        self._sortdec = rev
        member = Message()._messagemembers[col]
        self._messagelist = sorted(self._messagelist, key=lambda message: getattr(message, member).lower(), reverse=rev)

