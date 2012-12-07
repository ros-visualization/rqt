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

from .icon_tool_button import IconToolButton


class BatteryDashWidget(IconToolButton):
    """
    A Widget which displays incremental battery state, including a status tip.
    To use this widget simply call `update_perc` and `update_time` to change the displayed charge percentage and time remaining, respectively.

    :param name: The name of this widget
    :type name: str
    """
    def __init__(self, name='Battery', icons=None, charge_icons=None, icon_paths=None):
        if icons == None:
            icons = []
            charge_icons = []
            for x in range(6):
                icons.append(['ic-battery-%s.svg' % (x * 20)])
                charge_icons.append(['ic-battery-charge-%s.svg' % (x * 20)])
        super(BatteryDashWidget, self).__init__(name, icons, charge_icons, icon_paths=icon_paths)
        self.setEnabled(False)

        self._charge_icons = self._clicked_icons
        self.setStyleSheet('QToolButton:disabled {}')

        self._charging = False

        self.update_perc(0)

    def set_charging(self, value):
        self._charging = value

    def update_perc(self, val):
        """Update the displayed battery percentage.
        The default implementation of this method displays in 20% increments

        :param val: The new value to be displayed.
        :type val: int
        """
        self.update_state(round(val / 20.0))

    def _update_state(self, state):
        if self._charging:
            self.setIcon(self._charge_icons[state])
        else:
            self.setIcon(self._icons[state])

    def update_time(self, value):
        self.setToolTip("%s%% remaining" % value)
