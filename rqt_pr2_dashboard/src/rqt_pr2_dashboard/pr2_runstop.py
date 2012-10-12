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

from rqt_robot_dashboard.widgets import IconToolButton


class PR2Runstop(IconToolButton):
    def __init__(self, context):
        super(PR2Runstop, self).__init__('Runstop', [], [], 'nav.png', 'nav.png')
        
        self.setToolTip('Runstop')
        self.set_stale()

    def set_ok(self):
        self.update_state(0)

    def set_warn(self):
        self.update_state(1)

    def set_error(self):
        self.update_state(2)

    def set_stale(self):
        self.update_state(3)


class PR2WirelessRunstop(IconToolButton):
    def __init__(self, context):
        super(PR2WirelessRunstop, self).__init__('Wireless Runstop', [], [], 'nav-click.png', 'nav-click.png')
        
        self.setToolTip('Wireless Runstop')
        self.set_stale()

    def set_ok(self):
        self.update_state(0)

    def set_warn(self):
        self.update_state(1)

    def set_error(self):
        self.update_state(2)

    def set_stale(self):
        self.update_state(3)

