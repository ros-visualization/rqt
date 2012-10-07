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

import rospy
from robot_dashboard.widgets import BatteryDashWidget

from pr2_msgs.msg import PowerState, PowerBoardState
from pr2_power_board.srv import PowerBoardCommand, PowerBoardCommandRequest


class PR2Battery(BatteryDashWidget):
    def __init__(self, context):
        super(PR2Battery, self).__init__(context)
        self._power_consumption = 0.0
        self._pct = 0
        self._time_remaining = rospy.rostime.Duration(0)
        self._ac_present = 0
        self._plugged_in = False

    def set_power_state(self, msg):
        last_pct = self._pct
        last_plugged_in = self._plugged_in
        last_time_remaining = self._time_remaining
  
        self._power_consumption = msg.power_consumption
        self._time_remaining = msg.time_remaining
        self._pct = msg.relative_capacity / 100.0
        self._plugged_in = msg.AC_present
    
        if (last_pct != self._pct or last_plugged_in != self._plugged_in or last_time_remaining != self._time_remaining):
            drain_str = "remaining"
            if (self._plugged_in):
                drain_str = "to full charge"
            self.setToolTip("Battery: %.2f%% (%d minutes %s)"%(self._pct * 100.0, self._time_remaining.to_sec()/60.0, drain_str))
            self.update_perc(msg.relative_capacity)

    def set_stale(self):
        self._plugged_in = 0
        self._pct = 0
        self._time_remaining = rospy.rostime.Duration(0)
        self._power_consumption = 0
        self.setToolTip("Battery: Stale")
