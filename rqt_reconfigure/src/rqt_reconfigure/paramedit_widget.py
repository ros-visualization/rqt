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
#
# Author: Isaac Saito, Ze'ev Klapow

import os

import dynamic_reconfigure.client
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
import rospkg
import rospy

from .dynreconf_client_widget import DynreconfClientWidget
from .param_editors import EditorWidget, BooleanEditor, DoubleEditor, EnumEditor, IntegerEditor, StringEditor
from .param_groups import GroupWidget
from .param_updater import ParamUpdater

class ParameditWidget(QWidget):
    def __init__(self):
        super(ParameditWidget, self).__init__()
        
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_reconfigure'), 'resource', 'paramedit_pane.ui')
        loadUi(ui_file, self)
                
        self._dynreconf_client = None

        self.destroyed.connect(self.close)  # func in mercurial?

    def show_reconf(self, node):
        """
        
        :type node:
        """
        rospy.logdebug('ParameditWidget.show str(node)=%s', str(node))

        dynreconf_client = None        
        try:
            dynreconf_client = dynamic_reconfigure.client.Client(str(node), 
                                                                  timeout=5.0)
        except rospy.exceptions.ROSException:
            rospy.logerr("Could not connect to %s" % node)
            #TODO(Isaac) Needs to show err msg on GUI too. 
            return
        finally:
            if self._dynreconf_client:
                self._dynreconf_client.close() #Close old GUI client.

        self._dynreconf_client = DynreconfClientWidget(dynreconf_client) 
        # Client gets renewed every time different node was clicked.

        self._paramedit_scrollarea.setWidget(self._dynreconf_client)
        self._paramedit_scrollarea.setWidgetResizable(True)

    def close(self):
        if self._dynreconf_client is not None:
            # Clear out the old widget
            self._dynreconf_client.close()
            self._dynreconf_client = None

            self._paramedit_scrollarea.deleteLater()
