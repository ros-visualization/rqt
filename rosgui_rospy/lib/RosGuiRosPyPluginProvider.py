# Copyright (c) 2011, Dirk Thomas, TU Darmstadt
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

import roslib
roslib.load_manifest('rosgui_rospy')
import rospy

from rosgui.CompositePluginProvider import CompositePluginProvider

import rosgui.QtBindingHelper #@UnusedImport
from QtCore import qWarning

try:
    from rosgui.RospkgPluginProvider import RospkgPluginProvider
    ActualRosPluginProvider = RospkgPluginProvider
except ImportError:
    from rosgui.RoslibPluginProvider import RoslibPluginProvider
    ActualRosPluginProvider = RoslibPluginProvider

class RosGuiRosPyPluginProvider(CompositePluginProvider):

    def __init__(self):
        super(RosGuiRosPyPluginProvider, self).__init__([ActualRosPluginProvider('rosgui', 'rosgui_rospy::Plugin')])
        self.setObjectName('RosGuiRosPyPluginProvider')

    def discover(self):
        master = None
        try:
            # check if master is available
            master = rospy.get_master().getSystemState()
        except Exception:
            qWarning('RosGuiRosPyPluginProvider.discover() could not find ROS master, all rospy-based plugins are disabled')
        else:
            # initialize ROS node
            rospy.init_node('rosgui_rospy_node', anonymous=True, disable_signals=True)

        descriptors = super(RosGuiRosPyPluginProvider, self).discover()

        if master is None:
            # mark all plugins as "not_available"
            for descriptor in descriptors:
                descriptor.attributes()['not_available'] = 'no ROS master found (start roscore before?)'

        return descriptors
