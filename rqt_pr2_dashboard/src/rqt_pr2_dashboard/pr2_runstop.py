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
from python_qt_binding.QtCore import QSize

from rqt_robot_dashboard.widgets import IconToolButton
from rqt_robot_dashboard.util import make_icon

class PR2Runstops(IconToolButton):
    def __init__(self, context):
        super(PR2Runstops, self).__init__('Runstop', [], [], 'nav.png', 'nav.png')

        self.setToolTip('Runstop')

        self._ok_icon = [self.find_image('bg-green.svg'), self.find_image('ic-runstop-off.svg')]
        self._physical_engaged_icon = [self.find_image('bg-red.svg'), self.find_image('ic-runstop-on.svg'), self.find_image('ol-err-badge.svg')]
        self._wireless_engaged_icon = [self.find_image('bg-red.svg'), self.find_image('ic-wireless-runstop-on.svg'), self.find_image('ol-warn-badge.svg')]
        self._stale_icon = [self.find_image('bg-grey.svg'), self.find_image( 'ic-runstop-off.svg'), self.find_image('ol-stale-badge.svg')]

        self._icons = [make_icon(self._ok_icon), make_icon(self._physical_engaged_icon), make_icon(self._wireless_engaged_icon), make_icon(self._stale_icon)]
        self._clicked_icons = self._icons
        self.set_stale()

        self.setFixedSize(self._icons[0].actualSize(QSize(50,30)))

    def set_ok(self):
        self.update_state(0)

    def set_physical_engaged(self):
        self.update_state(1)

    def set_wireless_engaged(self):
        self.update_state(2)

    def set_stale(self):
        self.update_state(3)

