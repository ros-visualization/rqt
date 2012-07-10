import os
from QtGui import QDialog, QDialogButtonBox
from QtCore import QDateTime, Signal
from qt_gui.qt_binding_helper import loadUi
from datetime import datetime

class TimeDialog(QDialog):
    ignore_button_clicked = Signal()
    def __init__(self):
        super(QDialog, self).__init__()
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'time_dialog.ui')
        loadUi(ui_file, self)
        def click_handler(button):
            if button == self.button_box.button(QDialogButtonBox.Ignore):
                self.ignore_button_clicked.emit()
                self.accept()
        self.button_box.clicked.connect(click_handler)
    
    def set_time(self, mintime=None, maxtime=None):
        if maxtime is None:
            time = datetime.now()
            self.min_dateedit.setDateTime(time)
            self.max_dateedit.setDateTime(time)
        else:
            dtime = QDateTime()
            dtime.setTime_t(int(mintime[:mintime.find('.')]))
            dtime = dtime.addMSecs(int(mintime[mintime.find('.')+1:mintime.find('.')+4]))
            self.min_dateedit.setDateTime(dtime)
            dtime = QDateTime()
            dtime.setTime_t(int(maxtime[:maxtime.find('.')]))
            dtime = dtime.addMSecs(int(maxtime[maxtime.find('.')+1:maxtime.find('.')+4]))
            self.max_dateedit.setDateTime(dtime)


