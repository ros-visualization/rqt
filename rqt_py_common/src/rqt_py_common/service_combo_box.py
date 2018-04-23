#!/usr/bin/env python

# Copyright (c) 2011, Dorian Scholz, TU Darmstadt
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the TU Darmstadt nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy
import rosservice
from extended_combo_box import ExtendedComboBox
from python_qt_binding.QtCore import QStringListModel, QTimer

class ServiceComboBox(ExtendedComboBox):
    def __init__(self, parent=None, delay=500):
        super(ServiceComboBox, self).__init__(parent)
        self.setModel(QStringListModel(self.get_service_list()))
        self.update_timer = QTimer()
        self.update_timer.setInterval(delay)
        self.update_timer.timeout.connect(self.update)
        self.update_timer.start()

    def get_service_list(self):
        return sorted(set(rosservice.get_service_list()))

    def update(self):
        currentText = self.currentText()
        self.model().setStringList(self.get_service_list())
        self.setCurrentText(currentText)

if __name__ == "__main__":
    import sys
    from python_qt_binding.QtWidgets import QApplication

    app = QApplication(sys.argv)

    # Create the combo box itself.
    combo = ServiceComboBox()

    # Make sure your combo box is 
    combo.resize(600, 40)
    combo.show()

    sys.exit(app.exec_())
