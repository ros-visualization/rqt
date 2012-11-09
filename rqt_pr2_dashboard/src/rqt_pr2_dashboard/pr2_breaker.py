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
import os.path
import rospy

from python_qt_binding.QtCore import QSize

from pr2_msgs.msg import PowerBoardState
from pr2_power_board.srv import PowerBoardCommand, PowerBoardCommandRequest

from rqt_robot_dashboard.widgets import MonitorDashWidget, ConsoleDashWidget, MenuDashWidget, BatteryDashWidget, IconToolButton

from python_qt_binding.QtGui import QMessageBox


class PR2BreakerButton(MenuDashWidget):
    """
    Dashboard widget to display and interact with the PR2 Breaker state.
    """
    def __init__(self, breaker_name, breaker_index):
        """
        :param breaker_name: Name of the breaker
        :type breaker_name: str
        :param breaker_index: Index of the breaker
        :type breaker_index: int
        """

        import rospkg
        import os.path
        rp = rospkg.RosPack()

        if breaker_name == 'Left Arm':
            breaker_icon = 'ic-larm.svg'
        elif breaker_name == 'Base':
            breaker_icon = 'ic-base.svg'
        elif breaker_name == 'Right Arm':
            breaker_icon = 'ic-rarm.svg'
        else:
            breaker_icon = 'ic-breaker.svg'

        ok_icon = ['bg-green.svg', breaker_icon]
        warn_icon = ['bg-yellow.svg', breaker_icon, 'ol-warn-badge.svg']
        err_icon = ['bg-red.svg', breaker_icon, 'ol-err-badge.svg']
        stale_icon = ['bg-grey.svg', breaker_icon, 'ol-stale-badge.svg']

        icons = [ok_icon, warn_icon, err_icon, stale_icon]

        super(PR2BreakerButton, self).__init__('Breaker:' + breaker_name, icons=icons, icon_paths=[['rqt_pr2_dashboard','images']])
        self.update_state(3)

        self.setFixedSize(self._icons[0].actualSize(QSize(50,30)))

        self.add_action('Enable', self.on_enable)
        self.add_action('Standby', self.on_standby)
        self.add_action('Disable', self.on_standby)
        self.add_separator()
        self.add_action('Enable All Breakers', self.on_enable_all)
        self.add_action('Standby All Breakers', self.on_standby_all)
        self.add_action('Disable All Breakers', self.on_standby_all)

        self._power_control = rospy.ServiceProxy('power_board/control', PowerBoardCommand)
        self._serial = 0
        self._index = breaker_index
        self._name = breaker_name
        self._power_board_state = None
        self._last_status_msg = None
        self.setToolTip(breaker_name)

    def control(self, breaker, cmd):
        """
        Sends a PowerBoardCommand srv to the pr2

        :param breaker: breaker index to send command to
        :type breaker: int
        :param cmd: command to be sent to the pr2 breaker
        :type cmd: str
        """
        if (not self._power_board_state):
            QMessageBox.critical(self, "Error", self.tr("Cannot control breakers until we have received a power board state message"))
            return False

        if (not self._power_board_state.run_stop or not self._power_board_state.wireless_stop):
            if (cmd == "start"):
                QMessageBox.critical(self, "Error", self.tr("Breakers will not enable because one of the runstops is pressed"))
                return False
    
        try:
            power_cmd = PowerBoardCommandRequest()
            power_cmd.breaker_number = breaker
            power_cmd.command = cmd
            power_cmd.serial_number = self._serial
            self._power_control(power_cmd)
      
            return True
        except rospy.ServiceException, e:
            QMessageBox.critical(self, "Error", "Service call failed with error: %s"%(e), "Error")
            return False
      
        return False
  
    def control3(self, cmd):
        if (not self.control(0, cmd)):
            return False
        if (not self.control(1, cmd)):
            return False
        if (not self.control(2, cmd)):
            return False
        return True
  
    def on_enable(self):
        self.set_enable()
  
    def on_standby(self):
        self.set_standby()
  
    def on_disable(self):
        self.set_disable()
    
    def on_enable_all(self):
        self.set_enable_all()
  
    def on_standby_all(self):
        self.set_standby_all()
  
    def on_disable_all(self):
        self.set_disable_all()
    
    def set_enable(self):
        if (not self.control(self._index, "reset")):
            return
    
        self.control(self._index, "start")
    
    def set_standby(self):
        if (not self.control(self._index, "reset")):
            return
    
        self.control(self._index, "stop")
    
    def set_disable(self):
        self.control(self._index, "disable")
    
    def set_enable_all(self):
        if (not self.control3("reset")):
            return
    
        self.control3("start")
  
    def set_standby_all(self):
        if (not self.control3("reset")):
            return
    
        self.control3("stop")
  
    def set_disable_all(self):
        self.control3("disable")
    
    def set_power_board_state_msg(self, msg):
        """
        Sets state of button based on msg

        :param msg: message containing the PR2 powerboard state
        :type msg: pr2_msgs.msg.PowerBoardState
        """
        last_voltage = msg.circuit_voltage[self._index]
      
        self._power_board_state = msg
        self._serial = msg.serial_num
    
        status_msg = "OK"
    
        if (msg.circuit_state[self._index] == PowerBoardState.STATE_DISABLED):
            self.set_error()
            status_msg = "Disabled"
        elif (msg.circuit_state[self._index] == PowerBoardState.STATE_NOPOWER):
            self.set_error()
            status_msg = "No Power"
        elif (msg.circuit_state[self._index] == PowerBoardState.STATE_STANDBY):
            self.set_warn()
            status_msg = "Standby"
        else:
            self.set_ok()
     
        if (status_msg != self._last_status_msg or abs(last_voltage - msg.circuit_voltage[self._index]) >= 0.1):  
            self.setToolTip("Breaker: %s \nVoltage: %.02f \nState: %s"%(self._name, msg.circuit_voltage[self._index], status_msg))
            self._last_status_msg = status_msg
    
    def reset(self):
        self.set_stale()
        self.setToolTip("Breaker: %s (Stale)"%(self._name))

    def set_ok(self):
        self.update_state(0)

    def set_warn(self):
        self.update_state(1)

    def set_error(self):
        self.update_state(2)

    def set_stale(self):
        self.update_state(3)
