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
import threading
import time

import rospy

from qt_gui.composite_plugin_provider import CompositePluginProvider
from qt_gui.errors import PluginLoadError

from python_qt_binding.QtCore import qDebug, Qt, qWarning, Signal
from python_qt_binding.QtGui import QMessageBox

from rqt_gui.rospkg_plugin_provider import RospkgPluginProvider


class RosPyPluginProvider(CompositePluginProvider):

    _master_found_signal = Signal(int)

    def __init__(self):
        super(RosPyPluginProvider, self).__init__([RospkgPluginProvider('rqt_gui', 'rqt_gui_py::Plugin')])
        self.setObjectName('RosPyPluginProvider')
        self._node_initialized = False
        self._wait_for_master_dialog = None
        self._wait_for_master_thread = None

    def load(self, plugin_id, plugin_context):
        self._check_for_master()
        self._init_node()
        return super(RosPyPluginProvider, self).load(plugin_id, plugin_context)

    def _check_for_master(self):
        # check if master is available
        try:
            rospy.get_master().getSystemState()
            return
        except Exception:
            pass
        # spawn thread to detect when master becomes available
        self._wait_for_master_thread = threading.Thread(target=self._wait_for_master)
        self._wait_for_master_thread.start()
        self._wait_for_master_dialog = QMessageBox(QMessageBox.Question, self.tr('Waiting for ROS master'), self.tr("Could not find ROS master. Either start a 'roscore' or abort loading the plugin."), QMessageBox.Abort)
        self._master_found_signal.connect(self._wait_for_master_dialog.done, Qt.QueuedConnection)
        button = self._wait_for_master_dialog.exec_()
        # check if master existence was not detected by background thread
        no_master = button != QMessageBox.Ok
        self._wait_for_master_dialog.deleteLater()
        self._wait_for_master_dialog = None
        if no_master:
            raise PluginLoadError('RosPyPluginProvider._init_node() could not find ROS master')

    def _wait_for_master(self):
        while True:
            time.sleep(0.1)
            if not self._wait_for_master_dialog:
                break
            try:
                rospy.get_master().getSystemState()
            except Exception:
                continue
            self._master_found_signal.emit(QMessageBox.Ok)
            break

    def _init_node(self):
        # initialize node once
        if not self._node_initialized:
            name = 'rqt_gui_py_node_%d' % os.getpid()
            qDebug('RosPyPluginProvider._init_node() initialize ROS node "%s"' % name)
            rospy.init_node(name, disable_signals=True)
            self._node_initialized = True
