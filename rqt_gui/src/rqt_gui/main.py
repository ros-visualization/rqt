#!/usr/bin/env python

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

import os
import sys

import rospy
from rospkg.rospack import RosPack

from qt_gui.main import Main as Base


class Main(Base):

    def __init__(self, filename=None, ros_pack=None):
        rp = ros_pack or RosPack()
        qtgui_path = rp.get_path('qt_gui')
        super(Main, self).__init__(qtgui_path, invoked_filename=filename, settings_filename='rqt_gui')
        self._ros_pack = rp

    def main(self, argv=None, standalone=None, plugin_argument_provider=None):
        if argv is None:
            argv = sys.argv

        # ignore ROS specific remapping arguments (see http://www.ros.org/wiki/Remapping%20Arguments)
        argv = rospy.myargv(argv)

        return super(Main, self).main(argv, standalone=standalone, plugin_argument_provider=plugin_argument_provider, plugin_manager_settings_prefix=str(hash(os.environ['ROS_PACKAGE_PATH'])))

    def create_application(self, argv):
        from python_qt_binding.QtGui import QIcon
        app = super(Main, self).create_application(argv)
        logo = os.path.join(self._ros_pack.get_path('rqt_gui'), 'resource', 'rqt.png')
        icon = QIcon(logo)
        app.setWindowIcon(icon)
        return app

    def _add_plugin_providers(self):
        # do not import earlier as it would import Qt stuff without the proper initialization from qt_gui.main
        from qt_gui.recursive_plugin_provider import RecursivePluginProvider
        from .rospkg_plugin_provider import RospkgPluginProvider
        RospkgPluginProvider.rospack = self._ros_pack
        self.plugin_providers.append(RospkgPluginProvider('qt_gui', 'qt_gui_py::Plugin'))
        self.plugin_providers.append(RecursivePluginProvider(RospkgPluginProvider('qt_gui', 'qt_gui_py::PluginProvider')))
        self.plugin_providers.append(RecursivePluginProvider(RospkgPluginProvider('rqt_gui', 'rqt_gui_py::PluginProvider')))

    def _add_reload_paths(self, reload_importer):
        super(Main, self)._add_reload_paths(reload_importer)
        reload_importer.add_reload_path(os.path.join(os.path.dirname(__file__), *('..',) * 4))


if __name__ == '__main__':
    main = Main()
    sys.exit(main.main())
