import os
from qt_gui.qt_binding_helper import loadUi
from QtGui import QDialog

class ListDialog(QDialog):
    def __init__(self, windowtitle, text, boxlist, selected=''):
        super(QDialog, self).__init__()
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'list_dialog.ui')
        loadUi(ui_file, self)
        self.list_box.addItems(boxlist)
        for index, item in enumerate(boxlist):
            if selected.find(item) != -1: 
                self.list_box.item(index).setSelected(True)
        self.setWindowTitle(windowtitle)

    @staticmethod
    def show(titletext, labeltext, itemlist, selectedlist):
        dlg = ListDialog(titletext, labeltext, itemlist, selectedlist)
        ok = dlg.exec_()
        ok = (ok == 1)
        textlist = dlg.list_box.selectedItems()
        retlist = []
        for item in textlist:
            retlist.append(item.text())
        return (retlist, ok)


