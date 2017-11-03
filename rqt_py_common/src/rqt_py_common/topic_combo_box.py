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
#import rostopic
from extended_combo_box import ExtendedComboBox
from python_qt_binding.QtCore import QStringListModel

class TopicComboBox(ExtendedComboBox):
    def __init__(self, parent=None, delay=500):
        super(TopicComboBox, self).__init__(parent)
        self.setModel(QStringListModel(self.get_action_list()))
        self.update_timer = QTimer()
        self.update_timer.setInterval(delay)
        self.update_timer.timeout.connect(self.update)
        self.update_timer.start()

    def get_topic_list(self):
        # TO-DO: Replace with rostopic.get_topic_list() when ros/ros_comm#1154 is merged.
        # In the meantime this copies code from there.
        import rosgraph
        pubs, subs, _ = rosgraph.Master('/rostopic').getSystemState()
        pubs_out = []
        for topic, nodes in pubs:
            pubs_out.append((topic, "", nodes))
        subs_out = []
        for topic, nodes in subs:
            subs_out.append((topic, "", nodes))
        return (pubs_out, subs_out)

    def update(self):
        currentText = self.currentText()
        pubs, subs = get_topic_list()
        topics = sorted(set([x for x,_,_ in pubs] + [x for x,_,_ in subs]))
        combo.setModel(QStringListModel(topics))
        self.setCurrentText(currentText)

if __name__ == "__main__":
    import sys
    from python_qt_binding.QtWidgets import QApplication

    app = QApplication(sys.argv)

    # Create the combo box itself.
    combo = TopicComboBox()

    # Make sure your combo box is 
    combo.resize(600, 40)
    combo.show()

    sys.exit(app.exec_())
