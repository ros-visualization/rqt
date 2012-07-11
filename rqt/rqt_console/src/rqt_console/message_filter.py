from message import Message
from message_list import MessageList

class MessageFilter:
    """
    Filter object to hold data needed to filter a messagelist
    """
    def __init__(self):
        self._enabled = True
        self._filtertext = ''
        self._include = True

        # These apply locations will be unioned together
        self._applys = []  # list of which _messagemembers this filter applys to
        for member in MessageList().message_members():
            self._applys.append((member, True))
        self._applys = dict(self._applys)

