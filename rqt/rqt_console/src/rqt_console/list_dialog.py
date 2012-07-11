# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import os
from qt_gui.qt_binding_helper import loadUi
from QtGui import QDialog

class ListDialog(QDialog):
    """
    Simple dialog box that contains a QListWidget. The static method show
    is used to display the dialog and set the values
    """
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
    def show(titletext, labeltext, itemlist, selectedtext=''):
        """
        The item list will be inserted into the QListWidget and 
        an item will be selected if its text is present in selectedtext
        """
        dlg = ListDialog(titletext, labeltext, itemlist, selectedtext)
        ok = dlg.exec_()
        ok = (ok == 1)
        textlist = dlg.list_box.selectedItems()
        retlist = []
        for item in textlist:
            retlist.append(item.text())
        return (retlist, ok)


