from rosgraph_msgs.msg import Log 

from message import Message

class MessageList(object):
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

    def add_message(self, msg):
        self._messagelist.append(Message(msg))

    def header_print(self):
        return Message.header_print()
