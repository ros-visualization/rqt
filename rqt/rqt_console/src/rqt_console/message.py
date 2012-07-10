import qt_gui.qt_binding_helper  #@ UnusedImport

from QtCore import QObject
from rosgraph_msgs.msg import Log 

class Message(QObject):
    def __init__(self, msg=None):
        super(Message, self).__init__()
        self._messagemembers = self.get_message_members()
        self._severity = {1: self.tr('Debug'), 2: self.tr('Info'), 4:self.tr('Warn'), 8:self.tr('Error'), 16: self.tr('Fatal')}
        if msg is not None:
            self._message = msg.msg
            self._severity = self._severity[msg.level]
            self._node = msg.name
            self._time = (msg.header.stamp.secs, msg.header.stamp.nsecs)
            self._topics = ', '.join(msg.topics)
            self._location = msg.file + ':' + msg.function + ':' + str(msg.line)
    
    @staticmethod
    def get_message_members():
        return ('_message', '_severity', '_node', '_time', '_topics', '_location')

    @staticmethod
    def header_print():
        members = Message.get_message_members()
        text = members[2][1:].capitalize() + ';'
        text += members[3][1:].capitalize() + ';'
        text += members[1][1:].capitalize() + ';'
        text += members[4][1:].capitalize() + ';'
        text += members[5][1:].capitalize() + ';'
        text += members[0][1:].capitalize() + '\n'
        return text

    def count(self):
        return len(self._messagemembers)

    def time_in_seconds(self):
        return str(self._time[0]) + '.' + str(self._time[1]).zfill(9)

    def load_from_array(self, rowdata):
        self._message = rowdata[0]
        self._severity = rowdata[1]
        self._node = rowdata[2]
        self._time = rowdata[3].split('.')
        self._topics = rowdata[4]
        self._location = rowdata[5]
        return

    def file_print(self):
        text = self._node + ';'
        text += self.time_in_seconds() + ';'
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
        sec, nsec = text[:sc_index].split('.')
        self._time = (sec, nsec)
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
        text = self.tr('Node: ') + self._node + '\n'
        text += self.tr('Time: ') + self.time_in_seconds() + '\n'
        text += self.tr('Severity: ') + self._severity + '\n'
        text += self.tr('Published Topics: ') + self._topics + '\n'
        text += '\n' + self._message + '\n\n'
        return text


