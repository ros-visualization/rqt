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

import roslib;roslib.load_manifest('rqt_pr2_dashboard')
import rospy

import diagnostic_msgs

from pr2_msgs.msg import PowerState, PowerBoardState, DashboardState
from pr2_power_board.srv import PowerBoardCommand, PowerBoardCommandRequest
import std_msgs.msg
import std_srvs.srv

from robot_dashboard.dashboard import Dashboard
from robot_dashboard.widgets import MonitorDashWidget, ConsoleDashWidget 
from python_qt_binding.QtGui import QMessageBox

from .pr2_breaker import PR2BreakerButton
from .pr2_battery import PR2Battery
from .pr2_motors import PR2Motors
from .pr2_runstop import PR2Runstop, PR2WirelessRunstop


class PR2Dashboard(Dashboard):
    def setup(self, context):
        self.name = 'PR2 Dashboard'
        self.message = None

        self._dashboard_message = None
        self._last_dashboard_message_time = 0.0

        self._raw_byte = None
        self.digital_outs = [0,0,0]
        
        self._console = ConsoleDashWidget(self.context)
        self._monitor = MonitorDashWidget(self.context)
        self._motors = PR2Motors(self.context, self.on_reset_motors, self.on_halt_motors)
        self._breakers = [PR2BreakerButton('Left Arm', 0), 
                         PR2BreakerButton('Base', 1),
                         PR2BreakerButton('Right Arm', 2)]

        self._runstop = PR2Runstop('Run Stop')
        self._wireless_runstop = PR2WirelessRunstop('Wireless Run Stop')
        self._batteries = [PR2Battery(self.context)]

        self._dashboard_agg_sub = rospy.Subscriber('dashboard_agg', DashboardState, self.dashboard_callback)

    def get_widgets(self):
        return [[self._monitor, self._console , self._motors], self._breakers, [self._runstop, self._wireless_runstop], self._batteries]

    def dashboard_callback(self, msg):
        self._dashboard_message = msg
        self._last_dashboard_message_time = rospy.get_time()
      
        if (msg.motors_halted_valid):
            if (not msg.motors_halted.data):
                if (self._motors.set_ok()):
                    self._motors.setToolTip("Motors: Running")
            else:
                if (self._motors.set_error()):
                    self._motors.setToolTip("Motors: Halted")
        else:
            if (self._motors.set_stale()):
                self._motors.setToolTip("Motors: Stale")
        if (msg.power_state_valid):
            self._batteries[0].set_power_state(msg.power_state)
        else:
            self._batteries[0].set_stale()

        if (msg.power_board_state_valid):
            [breaker.set_power_board_state_msg(msg.power_board_state) for breaker in self._breakers]

            if (not msg.power_board_state.run_stop):
                # if the wireless stop is also off, we can't tell if the runstop is pressed or not
                if (not msg.power_board_state.wireless_stop):
                    if (self._runstop.set_warn()):
                        self._runstop.setToolTip("Physical Runstop: Unknown (Wireless is Pressed)")
                else:
                    if (self._runstop.set_error()):
                        self._runstop.setToolTip("Physical Runstop: Pressed")
            else:          
                if (self._runstop.set_ok()):
                    self._runstop.setToolTip("Physical Runstop: OK")
          
            if (not msg.power_board_state.wireless_stop):
                if (self._wireless_runstop.set_error()):
                    self._wireless_runstop.setToolTip("Wireless Runstop: Pressed")
            else:
                if (self._wireless_runstop.set_ok()):
                    self._wireless_runstop.setToolTip("Wireless Runstop: OK")
        else:
            if (self._wireless_runstop.setToolTip("Wireless Runstop: Stale")):
                self._runstop.setToolTip("Physical Runstop: Stale")
                [breaker.reset() for breaker in self._breakers]
                self._runstop.set_stale()
                self._wireless_runstop.set_stale()

    def on_reset_motors(self):
        # if any of the breakers is not enabled ask if they'd like to enable them
        if (self._dashboard_message is not None and self._dashboard_message.power_board_state_valid):
            all_breakers_enabled = reduce(lambda x,y: x and y, [state == PowerBoardState.STATE_ON for state in self._dashboard_message.power_board_state.circuit_state])
            if (not all_breakers_enabled):
                if(QMessageBox.question(self._breakers[0], 'Enable Breakers?', "Resetting the motors may not work because not all breakers are enabled.  Enable all the breakers first?",QMessageBox.Yes|QMessageBox.No,QMessageBox.Yes) == QMessageBox.Yes):
                    [breaker.set_enable() for breaker in self._breakers]
        reset = rospy.ServiceProxy("pr2_etherCAT/reset_motors", std_srvs.srv.Empty)
        try:
            reset()
        except rospy.ServiceException, e:
            QMessageBox.critical(self, "Error", "Failed to reset the motors: service call failed with error: %s"%(e))


    def on_halt_motors(self):
      halt = rospy.ServiceProxy("pr2_etherCAT/halt_motors", std_srvs.srv.Empty)
      try:
        halt()
      except rospy.ServiceException, e:
        QMessageBox.critical(self, "Error", "Failed to halt the motors: service call failed with error: %s"%(e))


