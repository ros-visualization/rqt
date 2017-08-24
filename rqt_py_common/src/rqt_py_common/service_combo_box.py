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
from python_qt_binding.QtCore import QStringListModel

class ServiceComboBox(ExtendedComboBox):
    def __init__(self, parent=None):
        super(ServiceComboBox, self).__init__(parent)
        # I attempted to create a timer to update the service list automatically,
        # but the timer runs in a different thread and PyQt is not thread-safe.
        # A ROS node must also be initialized for rospy to function.
        #self.update_timer = rospy.Timer(rospy.Duration.from_sec(0.5), self.on_update)

    def update_list(self):
        combo.setModel(QStringListModel(sorted(set(rosservice.get_service_list()))))

if __name__ == "__main__":
    import sys
    from python_qt_binding.QtWidgets import QApplication

    app = QApplication(sys.argv)

    # Create the combo box itself.
    combo = ServiceComboBox()
    # Clear the list of services and pull a new one. Do this on a regular basis, such as when
    # a user changes an option or clicks a Refresh button.
    combo.update_list()

    # Make sure your combo box is 
    combo.resize(600, 40)
    combo.show()

    sys.exit(app.exec_())
