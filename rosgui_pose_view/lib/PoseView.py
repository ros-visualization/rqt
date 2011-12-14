# Copyright (c) 2011, Dorian Scholz, TU Darmstadt
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

import rosgui.QtBindingHelper #@UnusedImport
from QtCore import QObject, Qt
from QtGui import QDockWidget

import roslib
roslib.load_manifest('rosgui_pose_view')

import PoseViewWidget
reload(PoseViewWidget) # force reload to update on changes during runtime

class PoseView(QObject):

    def __init__(self, parent, plugin_context):
        super(PoseView, self).__init__(parent)
        self.setObjectName('PoseView')

        self._widget = PoseViewWidget.PoseViewWidget(self, plugin_context)
        self.set_name('Pose View')

        if plugin_context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % plugin_context.serial_number()))

        # add _widget to the main window
        plugin_context.main_window().addDockWidget(Qt.RightDockWidgetArea, self._widget)


    def set_name(self, name):
        self._widget.setWindowTitle(name)


    def save_settings(self, global_settings, perspective_settings):
        self._widget.save_settings(global_settings, perspective_settings)


    def restore_settings(self, global_settings, perspective_settings):
        self._widget.restore_settings(global_settings, perspective_settings)


    def close_plugin(self):
        QDockWidget.close(self._widget)
        self._widget.deleteLater()
