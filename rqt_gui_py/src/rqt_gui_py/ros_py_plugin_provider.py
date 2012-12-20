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

import os
import rospy

from qt_gui.composite_plugin_provider import CompositePluginProvider

from python_qt_binding.QtCore import qDebug, qWarning

try:
    from rqt_gui.rospkg_plugin_provider import RospkgPluginProvider
    ActualRosPluginProvider = RospkgPluginProvider
except ImportError:
    from rqt_gui.roslib_plugin_provider import RoslibPluginProvider
    ActualRosPluginProvider = RoslibPluginProvider


class RosPyPluginProvider(CompositePluginProvider):

    def __init__(self):
        super(RosPyPluginProvider, self).__init__([ActualRosPluginProvider('rqt_gui', 'rqt_gui_py::Plugin')])
        self.setObjectName('RosPyPluginProvider')
        self._node_initialized = False

    def discover(self):
        descriptors = super(RosPyPluginProvider, self).discover()

        if not self._master_found():
            qWarning('RosPyPluginProvider.discover() could not find ROS master, all rospy-based plugins are disabled')
            # mark all plugins as "not_available"
            for descriptor in descriptors:
                descriptor.attributes()['not_available'] = 'no ROS master found (roscore needs to be started before the GUI)'

        return descriptors

    def load(self, plugin_id, plugin_context):
        self._init_node()
        return super(RosPyPluginProvider, self).load(plugin_id, plugin_context)

    def _master_found(self):
        try:
            rospy.get_master().getSystemState()
            return True
        except Exception:
            return False

    def _init_node(self):
        # initialize node once
        if not self._node_initialized:
            name = 'rqt_gui_py_node_%d' % os.getpid()
            qDebug('RosPyPluginProvider._init_node() initialize ROS node "%s"' % name)
            rospy.init_node(name, disable_signals=True)
            self._node_initialized = True
