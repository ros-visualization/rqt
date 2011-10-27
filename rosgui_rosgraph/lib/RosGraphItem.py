import rosgui.QtBindingHelper #@UnusedImport
from QtGui import QColor, QGraphicsItemGroup

class RosGraphItem(QGraphicsItemGroup):

    _COLOR_BLACK = QColor(0, 0, 0)
    _COLOR_BLUE = QColor(0, 0, 204)
    _COLOR_GREEN = QColor(0, 170, 0)
    _COLOR_ORANGE = QColor(255, 165, 0)
    _COLOR_RED = QColor(255, 0, 0)
    _COLOR_TEAL = QColor(0, 170, 170)

    def __init__(self, highlight_level, parent=None):
        super(RosGraphItem, self).__init__(parent)
        self._highlight_level = highlight_level

