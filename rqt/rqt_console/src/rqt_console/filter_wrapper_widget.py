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

from QtGui import QIcon, QWidget
from qt_gui.qt_binding_helper import loadUi
from QtCore import QRegExp, Qt

class FilterWrapperWidget(QWidget):
    def __init__(self, widget, filter_name):
        super(FilterWrapperWidget, self).__init__()
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'filter_wrapper_widget.ui')
        loadUi(ui_file, self)
        self.setObjectName('FilterWrapperWidget')
        self.delete_button.setIcon(QIcon.fromTheme('list-remove'))
        stretch = self.layout_frame.stretch(2)
        self.layout_frame.insertWidget(2, widget)
        self.layout_frame.setStretch(2, stretch)
        # Hack to hide the placeholder widget since removing it caused problems
        self.layout_frame.setStretch(3, 0)
#TODO why does this call make other widgets stop working?
#        self.layout_frame.removeWidget(self.layout_frame.itemAt(2).widget())
        self._widget = widget
        self.enabled_checkbox.stateChanged[int].connect(self.enabled_callback)
        self.filter_name_label.setText(filter_name + ':')

    def enabled_callback(self, checked):
        self._widget._parentfilter.set_enabled(checked)
    
    def enable_all_children(self):
        for child in self.findChildren(QWidget, QRegExp('.*')):
            # TODO this is not enabling 
            #print 'calling setEnabled on: ', child
            child.setEnabled(True)

    def repopulate(self):
        self.findChildren(QWidget, QRegExp('.*FilterWidget.*'))[0].repopulate()
