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

import argparse
import diagnostic_msgs

from pr2_msgs.msg import PowerState, PowerBoardState, DashboardState
from pr2_power_board.srv import PowerBoardCommand, PowerBoardCommandRequest
import std_msgs.msg
import std_srvs.srv

from rqt_robot_dashboard.dashboard import Dashboard
from rqt_robot_dashboard.widgets import MonitorDashWidget, ConsoleDashWidget 

from python_qt_binding.QtCore import QSize
from python_qt_binding.QtGui import QMessageBox

from .pr2_breaker import PR2BreakerButton
from .pr2_battery import PR2Battery
from .pr2_motors import PR2Motors
from .pr2_runstop import PR2Runstops


class PR2Dashboard(Dashboard):
    """
    Dashboard for PR2s

    :param context: the plugin context
    :type context: qt_gui.plugin.Plugin
    """
    def setup(self, context):
        # argparse example
        parser = argparse.ArgumentParser()
        self._add_args(parser)
        parser.parse_args(context.argv())

        self.name = 'PR2 Dashboard'
        self.max_icon_size = QSize(50, 30)
        self.message = None

        self._dashboard_message = None
        self._last_dashboard_message_time = 0.0

        self._raw_byte = None
        self.digital_outs = [0,0,0]

        self._console = ConsoleDashWidget(self.context, minimal=False)
        self._monitor = MonitorDashWidget(self.context)
        self._motors = PR2Motors(self.on_reset_motors, self.on_halt_motors)
        self._breakers = [PR2BreakerButton('Left Arm', 0), 
                         PR2BreakerButton('Base', 1),
                         PR2BreakerButton('Right Arm', 2)]

        self._runstop = PR2Runstops('RunStops')
        self._batteries = [PR2Battery(self.context)]

        self._dashboard_agg_sub = rospy.Subscriber('dashboard_agg', DashboardState, self.dashboard_callback)

    def _add_args(self, parser):
        parser.add_argument("-u", "--ros_master_uri", help="Set the ROS_MASTER_URI")

    def get_widgets(self):
        return [[self._monitor, self._console , self._motors], self._breakers, [self._runstop], self._batteries]

    def dashboard_callback(self, msg):
        """
        callback to process dashboard_agg messages

        :param msg: dashboard_agg DashboardState message
        :type msg: pr2_msgs.msg.DashboardState
        """
        self._dashboard_message = msg
        self._last_dashboard_message_time = rospy.get_time()

        if (msg.motors_halted_valid):
            if (not msg.motors_halted.data):
                self._motors.set_ok()
                self._motors.setToolTip(self.tr("Motors: Running"))
            else:
                self._motors.set_error()
                self._motors.setToolTip(self.tr("Motors: Halted"))
        else:
            self._motors.set_stale()
            self._motors.setToolTip(self.tr("Motors: Stale"))

        if (msg.power_state_valid):
            self._batteries[0].set_power_state(msg.power_state)
        else:
            self._batteries[0].set_stale()

        if (msg.power_board_state_valid):
            [breaker.set_power_board_state_msg(msg.power_board_state) for breaker in self._breakers]
            if msg.power_board_state.run_stop:
                self._runstop.set_ok()
                self._runstop.setToolTip(self.tr("Physical Runstop: OK\nWireless Runstop: OK"))
            elif msg.power_board_state.wireless_stop:
                self._runstop.set_physical_engaged()
                self._runstop.setToolTip(self.tr("Physical Runstop: Pressed\nWireless Runstop: OK"))
            if not msg.power_board_state.wireless_stop:
                self._runstop.set_wireless_engaged()
                self._runstop.setToolTip(self.tr("Physical Runstop: Unknown\nWireless Runstop: Pressed"))
        else:
            [breaker.reset() for breaker in self._breakers]
            self._runstop.set_stale()
            self._runstop.setToolTip(self.tr("Physical Runstop: Stale\nWireless Runstop: Stale"))

    def on_reset_motors(self):
        # if any of the breakers is not enabled ask if they'd like to enable them
        if (self._dashboard_message is not None and self._dashboard_message.power_board_state_valid):
            all_breakers_enabled = reduce(lambda x,y: x and y, [state == PowerBoardState.STATE_ON for state in self._dashboard_message.power_board_state.circuit_state])
            if (not all_breakers_enabled):
                if(QMessageBox.question(self._breakers[0], self.tr('Enable Breakers?'), self.tr("Resetting the motors may not work because not all breakers are enabled.  Enable all the breakers first?"),QMessageBox.Yes|QMessageBox.No,QMessageBox.Yes) == QMessageBox.Yes):
                    [breaker.set_enable() for breaker in self._breakers]
        reset = rospy.ServiceProxy("pr2_etherCAT/reset_motors", std_srvs.srv.Empty)
        try:
            reset()
        except rospy.ServiceException, e:
            QMessageBox.critical(self._breakers[0], "Error", "Failed to reset the motors: service call failed with error: %s"%(e))


    def on_halt_motors(self):
      halt = rospy.ServiceProxy("pr2_etherCAT/halt_motors", std_srvs.srv.Empty)
      try:
        halt()
      except rospy.ServiceException, e:
        QMessageBox.critical(self._motors, "Error", "Failed to halt the motors: service call failed with error: %s"%(e))


