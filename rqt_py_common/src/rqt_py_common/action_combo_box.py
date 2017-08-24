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

class ActionComboBox(ExtendedComboBox):
    def __init__(self, parent=None):
        super(ActionComboBox, self).__init__(parent)
        # I attempted to create a timer to update the topic list automatically,
        # but the timer runs in a different thread and PyQt is not thread-safe.
        # A ROS node must also be initialized for rospy to function.
        #self.update_timer = rospy.Timer(rospy.Duration.from_sec(0.5), self.on_update)

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

    def update_list(self):
        pubs, subs = self.get_topic_list()
        topics = sorted(set([x for x,_,_ in pubs + subs]))
        # Action filter code from https://github.com/mcgill-robotics/rosaction/blob/master/src/rosaction/__init__.py#L151
        actions = [x[:-5] for x in topics if x.endswith("goal") and x.replace("/goal", "/cancel") in topics]
        combo.setModel(QStringListModel(actions))

if __name__ == "__main__":
    import sys
    from python_qt_binding.QtWidgets import QApplication

    app = QApplication(sys.argv)

    # Create the combo box itself.
    combo = ActionComboBox()
    # Clear the list of topics and pull a new one. Do this on a regular basis, such as when
    # a user changes an option or clicks a Refresh button.
    combo.update_list()

    # Make sure your combo box is 
    combo.resize(600, 40)
    combo.show()

    sys.exit(app.exec_())
