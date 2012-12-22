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

import rospy
from python_qt_binding.QtCore import QMutex, QMutexLocker, QSize, QTimer

from rqt_console.console_subscriber import ConsoleSubscriber
from rqt_console.console_widget import ConsoleWidget
from rqt_console.message_data_model import MessageDataModel
from rqt_console.message_proxy_model import MessageProxyModel

from .icon_tool_button import IconToolButton


class ConsoleDashWidget(IconToolButton):
    """
    A widget which brings up the ROS console.

    :param context: The plugin context to create the monitor in.
    :type context: qt_gui.plugin_context.PluginContext
    """
    def __init__(self, context, icon_paths=None, minimal=True):
        ok_icon = ['bg-green.svg', 'ic-console.svg']
        warn_icon = ['bg-yellow.svg', 'ic-console.svg', 'ol-warn-badge.svg']
        err_icon = ['bg-red.svg', 'ic-console.svg', 'ol-err-badge.svg']
        stale_icon = ['bg-grey.svg', 'ic-console.svg', 'ol-stale-badge.svg']

        icons = [ok_icon, warn_icon, err_icon, stale_icon]

        super(ConsoleDashWidget, self).__init__('Console Widget', icons, icon_paths=icon_paths)

        self.minimal = minimal
        self.setFixedSize(self._icons[0].actualSize(QSize(50, 30)))

        self._datamodel = MessageDataModel()
        self._proxymodel = MessageProxyModel()
        self._proxymodel.setSourceModel(self._datamodel)

        self._mutex = QMutex()
        self._subscriber = ConsoleSubscriber(self._message_cb)

        self._console = None
        self.context = context
        self.clicked.connect(self._show_console)

        self.update_state(0)
        self._timer = QTimer()
        self._timer.timeout.connect(self._insert_messages)
        self._timer.start(100)

        if self._console is None:
            self._console = ConsoleWidget(self._proxymodel, self.minimal)
            self._console.destroyed.connect(self._console_destroyed)
        self._console_shown = False
        self.setToolTip("Rosout")

    def _show_console(self):
        if self._console is None:
            self._console = ConsoleWidget(self._proxymodel, self.minimal)
            self._console.destroyed.connect(self._console_destroyed)
        try:
            if self._console_shown:
                self.context.remove_widget(self._console)
                self._console_shown = not self._console_shown
            else:
                self.context.add_widget(self._console)
                self._console_shown = not self._console_shown
        except Exception:
            self._console_shown = not self._console_shown
            self._show_console()

    def _insert_messages(self):
        with QMutexLocker(self._mutex):
            msgs = self._datamodel._insert_message_queue
            self._datamodel._insert_message_queue = []
        self._datamodel.insert_rows(msgs)

        # The console may not yet be initialized or may have been closed
        # So fail silently
        try:
            self.update_rosout()
            self._console.update_status()
        except:
            pass

    def _message_cb(self, msg):
        if not self._datamodel._paused:
            with QMutexLocker(self._mutex):
                self._datamodel._insert_message_queue.append(msg)

    def update_rosout(self):
        summary_dur = 30.0
        if (rospy.get_time() < 30.0):
            summary_dur = rospy.get_time() - 1.0

        if (summary_dur < 0):
            summary_dur = 0.0

        summary = self._console.get_message_summary(summary_dur)

        if (summary.fatal or summary.error):
            self.update_state(2)
        elif (summary.warn):
            self.update_state(1)
        else:
            self.update_state(0)

        tooltip = ""
        if (summary.fatal):
            tooltip += "\nFatal: %s" % (summary.fatal)
        if (summary.error):
            tooltip += "\nError: %s" % (summary.error)
        if (summary.warn):
            tooltip += "\nWarn: %s" % (summary.warn)
        if (summary.info):
            tooltip += "\nInfo: %s" % (summary.info)
        if (summary.debug):
            tooltip += "\nDebug: %s" % (summary.debug)

        if (len(tooltip) == 0):
            tooltip = "Rosout: no recent activity"
        else:
            tooltip = "Rosout: recent activity:" + tooltip

        if tooltip != self.toolTip():
            self.setToolTip(tooltip)

    def _console_destroyed(self):
        if self._console:
            self._console.cleanup_browsers_on_close()
        self._console = None

    def shutdown_widget(self):
        if self._console:
            self._console.cleanup_browsers_on_close()
        if self._subscriber:
            self._subscriber.unsubscribe_topic()
        self._timer.stop()

    def save_settings(self, plugin_settings, instance_settings):
        self._console.save_settings(plugin_settings, instance_settings)

    def restore_settings(self, plugin_settings, instance_settings):
        self._console.restore_settings(plugin_settings, instance_settings)
