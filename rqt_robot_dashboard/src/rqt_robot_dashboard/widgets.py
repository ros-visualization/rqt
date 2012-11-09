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

"""
.. module:: widgets
    :synopsis: Widgets for the rqt_robot_dashboard.

.. moduleauthor:: Ze'ev Klapow <zklapow@willowgarage.com>, Aaron Blasdel <ablasdel@willowgarage.com>

This module provides a set of standard widgets for using with the Dashboard class.

To use them you must provide instances of them to your dashboard in its :func:`get_widgets` method. For example::
    
    from rqt_robot_dashboard.dashboard import Dashboard
    from rqt_robot_dashboard.widgets import MonitorDashWidget, ConsoleDashWidget

    class MyDashboard(Dashboard):
        def get_widgets(self):
            self.monitor = MonitorDashWidget(self.context)
            self.console = ConsoleDashWidget(self.context)
            self.battery = BatteryDashWidget(self.context)

            return [[self.monitor, self.console],[self.battery]]

Would create a simple dashboard with the ability to open a rqt_robot_monitor and a ROS console and monitor the battery.
"""

import roslib;roslib.load_manifest('rqt_robot_dashboard')
import rospy
from rqt_robot_monitor import RobotMonitorWidget
from rqt_nav_view import NavViewWidget

from .util import IconHelper

from rqt_console.console_widget import ConsoleWidget
from rqt_console.console_subscriber import ConsoleSubscriber
from rqt_console.message_data_model import MessageDataModel
from rqt_console.message_proxy_model import MessageProxyModel
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

from QtCore import Signal, QMutex, QMutexLocker, QSize, QTimer
from QtGui import QPushButton, QMenu, QIcon, QWidget, QVBoxLayout, QColor, QProgressBar, QToolButton
import os.path


class IconToolButton(QToolButton):
    """
    This is the base class for all widgets. 
    It provides state and icon switching support as well as convenience functions for creating icons.

    :raises IndexError: if ``icons`` is not a list of lists of strings

    :param name: name of the object
    :type name: str
    :param icons: A list of lists of strings to create icons for the states of this button. 
    If only one is supplied then ok, warn, error, stale icons will be created with overlays
    :type icons: list
    :param clicked_icons: A list of clicked state icons. len must equal icons
    :type clicked_icons: list
    :param suppress_overlays: if false and there is only one icon path supplied
    :type suppress_overlays: bool
    :param icon_paths: list of lists of package and subdirectory in the form
    ['package name', 'subdirectory'] example ['rqt_pr2_dashboard', 'images/svg']
    :type icon_paths: list of lists of strings
    """
    state_changed = Signal(int)
    def __init__(self, name, icons, clicked_icons=None, suppress_overlays=False, icon_paths=[]):
        super(IconToolButton, self).__init__()

        self.name = name
        self.setObjectName(self.name)

        self.state_changed.connect(self._update_state)
        self.pressed.connect(self._pressed)
        self.released.connect(self._released)

        import rospkg
        icon_paths = icon_paths + [['rqt_robot_dashboard', 'images']]
        paths = []
        for path in icon_paths:
            paths.append(os.path.join(rospkg.RosPack().get_path(path[0]), path[1]))
        self.icon_helper = IconHelper(paths)

        self.setStyleSheet('QToolButton {border: none;}')

        self.__state = 0
        self.set_icon_lists(icons, clicked_icons, suppress_overlays)

    def update_state(self, state):
        """
        Set the state of this button. 
        This will also update the icon for the button based on the ``self._icons`` list

        :raises IndexError: If state is not a proper index to ``self._icons``

        :param state: The state to set.
        :type state: int
        """
        if 0 <= state and state < len(self._icons):
            self.__state = state
            self.state_changed.emit(self.__state)
        else:
            raise(IndexError("%s update_state received invalid state: %s"(self.name, state)))

    def set_icon_lists(self, icons, clicked_icons=None, suppress_overlays=False):
        """
        Sets up the icon lists for the button states.
        There must be one index in icons for each state.

        :raises IndexError: if ``icons`` is not a list of lists of strings

        :param icons: A list of lists of strings to create icons for the states of this button. 
        If only one is supplied then ok, warn, error, stale icons will be created with overlays
        :type icons: list
        :param clicked_icons: A list of clicked state icons. len must equal icons
        :type clicked_icons: list
        :param suppress_overlays: if false and there is only one icon path supplied
        :type suppress_overlays: bool

        """
        if clicked_icons is not None and len(icons) != len(clicked_icons):
            rospy.logerr("%s: icons and clicked states are unequal"%self.name)
            icons = clicked_icons = ['icon_not_found.svg']
        if not (type(icons) is list and type(icons[0]) is list and type(icons[0][0] is str)):
            raise(IndexError("icons must be a list of lists of strings"))
        if len(icons) <= 0:
            rospy.logerr("%s: Icons not supplied"%self.name)
            icons = clicked_icons = ['icon_not_found.svg']
        if len(icons) == 1 and suppress_overlays == False:
            if icons[0][0][-4].lower() == '.svg':
                icons.append(icons[0] + ['ol-warn-badge.svg'])
                icons.append(icons[0] + ['ol-err-badge.svg'])
                icons.append(icons[0] + ['ol-stale-badge.svg'])
            else:
                icons.append(icons[0] + ['warn-overlay.png'])
                icons.append(icons[0] + ['err-overlay.png'])
                icons.append(icons[0] + ['stale-overlay.png'])
        if clicked_icons is None:
            clicked_icons = []
            for name in icons:
                clicked_icons.append(name + ['ol-click.svg'])
        self._icons = []
        for icon in icons:
            self._icons.append(self.icon_helper.build_icon(icon))
        self._clicked_icons = []
        for icon in clicked_icons:
            self._clicked_icons.append(self.icon_helper.build_icon(icon))

    def _update_state(self, state):
        if self.isDown():
            self.setIcon(self._clicked_icons[self.__state])
        else:
            self.setIcon(self._icons[self.__state])

    def _pressed(self):
        self.setIcon(self._clicked_icons[self.__state])

    def _released(self):
        self.setIcon(self._icons[self.__state])

class MenuDashWidget(IconToolButton):
    """
    A widget which displays a pop-up menu when clicked

    :param name: The name to give this widget.
    :type name: str
    :param icon: The icon to display in this widgets button.
    :type icon: str
    """
    def __init__(self, name, icons=None, clicked_icons=None, icon_paths=[]):
        if icons == None:
            icons = [['ic-motors.svg']]
        super(MenuDashWidget, self).__init__(name, icons=icons, suppress_overlays=True, icon_paths=icon_paths)
        self.setStyleSheet('QToolButton::menu-indicator {image: url(none.jpg);} QToolButton {border: none;}')
        self.setPopupMode(QToolButton.InstantPopup)
        self.update_state(0)

        self.pressed.disconnect(self._pressed)
        self.released.disconnect(self._released)

        self._menu = QMenu()
        self._menu.aboutToHide.connect(self._released)
        self._menu.aboutToShow.connect(self._pressed)

        self.setMenu(self._menu)

    def add_separator(self):
        return self._menu.addSeparator()

    def add_action(self, name, callback):
        """
        Add an action to the menu, and return the newly created action.

        :param name: The name of the action.
        :type name: str
        :param callback: Function to be called when this item is pressed.
        :type callback: callable
        """
        return self._menu.addAction(name, callback)

class MonitorDashWidget(IconToolButton):
    """
    A widget which brings up the rqt_robot_monitor.

    :param context: The plugin context to create the monitor in.
    :type context: qt_gui.plugin_context.PluginContext
    """
    def __init__(self, context, icon_paths=[]):
        ok_icon = ['bg-green.svg', 'ic-diagnostics.svg']
        warn_icon = ['bg-yellow.svg', 'ic-diagnostics.svg', 'ol-warn-badge.svg']
        err_icon = ['bg-red.svg', 'ic-diagnostics.svg', 'ol-err-badge.svg']
        stale_icon = ['bg-grey.svg', 'ic-diagnostics.svg', 'ol-stale-badge.svg']

        icons = [ok_icon, warn_icon, err_icon, stale_icon]

        super(MonitorDashWidget, self).__init__('MonitorWidget', icons, icon_paths=icon_paths)

        self.update_state(2)

        self.setFixedSize(self._icons[0].actualSize(QSize(50, 30)))

        self._monitor = None
        self._close_mutex = QMutex()

        self._last_update = rospy.Time.now()

        self.context = context
        self.clicked.connect(self._show_monitor)

        self.update_state(3)

        self._monitor_shown = False
        self.setToolTip('Diagnostics')

        self._diagnostics_toplevel_state_sub = rospy.Subscriber('diagnostics_toplevel_state', DiagnosticStatus, self.toplevel_state_callback)
        self._top_level_state = -1
        self._stall_timer = QTimer()
        self._stall_timer.timeout.connect(self._stalled)
        self._stalled()
        self._is_stale = True

    def toplevel_state_callback(self, msg):
        self._is_stale = False
        self._stall_timer.start(5000)

        if self._top_level_state != msg.level:
            if (msg.level >= 2):
                self.update_state(2)
                self.setToolTip("Diagnostics: Error")
            elif (msg.level == 1):
                self.update_state(1)
                self.setToolTip("Diagnostics: Warning")
            else:
                self.update_state(0)
                self.setToolTip("Diagnostics: OK")
            self._top_level_state = msg.level

    def _stalled(self):
        self._stall_timer.stop()
        self._is_stale = True
        self.update_state(3)
        self.setToolTip("Diagnostics: Stale\nNo message received on dashboard_agg in the last 5 seconds")

    def _show_monitor(self):
        try:
            if self._monitor_shown:
                self._monitor_shown = False
                self.context.remove_widget(self._monitor)
                self._monitor_close()
            else:
                self._monitor = RobotMonitorWidget(self.context, 'diagnostics_agg')
                self._monitor.destroyed.connect(self._monitor_close)
                self.context.add_widget(self._monitor)
                self._monitor_shown = True
        except Exception as e:
            #  TODO when closeEvents is available fix this hack (It ensures the button will toggle correctly)
            self._show_monitor()

    def _monitor_close(self):
        locker = QMutexLocker(self._close_mutex)
        if self._monitor_shown:
            self._monitor_shown = False
            self._monitor.close()
            self._monitor = None

    def close(self):
        self._monitor_close()

class ConsoleDashWidget(IconToolButton):
    """
    A widget which brings up the ROS console.

    :param context: The plugin context to create the monitor in.
    :type context: qt_gui.plugin_context.PluginContext
    """
    def __init__(self, context, icon_paths=[], minimal=True):
        ok_icon = ['bg-green.svg', 'ic-console.svg']
        warn_icon = ['bg-yellow.svg', 'ic-console.svg', 'ol-warn-badge.svg']
        err_icon = ['bg-red.svg', 'ic-console.svg', 'ol-err-badge.svg']
        stale_icon = ['bg-grey.svg', 'ic-console.svg', 'ol-stale-badge.svg']

        icons = [ok_icon, warn_icon, err_icon, stale_icon]

        super(ConsoleDashWidget, self).__init__('Console Widget', icons, icon_paths=icon_paths)
        self.update_state(3)

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

        self._timer = QTimer()
        self._timer.timeout.connect(self._insert_messages)
        self._timer.start(100)

        if self._console is None:
            self._console = ConsoleWidget(self._proxymodel, self.minimal)
            self._console.destroyed.connect(self._console_destroyed)
        self._console_shown = False
        self.setToolTip("Rosout")
        self.update_state(0)

    def _show_console(self):
        if self._console is None:
            self._console = ConsoleWidget(self._proxymodel, self.minimal)
            self._console.destroyed.connect(self._console_destroyed)
        try:
            if self._console_shown:
                self.context.remove_widget(self._console)
            else:
                self.context.add_widget(self._console)
        except Exception as e:
            pass
        finally:
            self._console_shown = not self._console_shown

 
    def _insert_messages(self):
        self._mutex.lock()
        msgs = self._datamodel._insert_message_queue
        self._datamodel._insert_message_queue = []
        self._mutex.unlock()
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
            self._mutex.lock()
            self._datamodel._insert_message_queue.append(msg)
            self._mutex.unlock()

    def _console_destroyed(self):
        self._console = None

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
            tooltip += "\nFatal: %s"%(summary.fatal)
        if (summary.error):
            tooltip += "\nError: %s"%(summary.error)
        if (summary.warn):
            tooltip += "\nWarn: %s"%(summary.warn)
        if (summary.info):
            tooltip += "\nInfo: %s"%(summary.info)
        if (summary.debug):
            tooltip += "\nDebug: %s"%(summary.debug)

        if (len(tooltip) == 0):
            tooltip = "Rosout: no recent activity"
        else:
            tooltip = "Rosout: recent activity:" + tooltip

        if (tooltip != self.toolTip()):
            self.setToolTip(tooltip)

class BatteryDashWidget(IconToolButton):
    """
    A Widget which displays incremental battery state, including a status tip.
    To use this widget simply call `update_perc` and `update_time` to change the displayed charge percentage and time remaining, respectively.

    :param context: The plugin context
    :type context: qt_gui.plugin_context.PluginContext
    :param name: The name of this widget
    :type name: str
    """
    def __init__(self, context, name='Battery', icons=None, charge_icons=None, icon_paths=[]):
        if icons == None:
            icons = []
            charge_icons = []
            for x in range(0, 6):
                icons.append(['ic-battery-%s.svg'%(x*20)])
                charge_icons.append(['ic-battery-charge-%s.svg'%(x*20)])
        super(BatteryDashWidget, self).__init__(name, icons, charge_icons, icon_paths=icon_paths)
        self.setEnabled(False)

        self.setStyleSheet('QToolButton:disabled {}')
        self.charging = False
        self.update_perc(0)

    def update_perc(self, val):
        """Update the displayed battery percentage. 
        The default implementation of this method displays in 20% increments

        :param val: The new value to be displayed.
        :type val: int
        """
        self.update_state(round(val/20.0))

    def _update_state(self, state):
        if self.charging:
            self.setIcon(self._charge_icons[state])
        else:
            self.setIcon(self._icons[state])

    def update_time(self, val):
        self.time_remaining = val
        self.setStatusTip("%s remaining"%val)

class NavViewDashWidget(IconToolButton):
    """
    A widget which launches a nav_view widget in order to view and interact with the ROS nav stack

    :param context: The plugin context in which to dsiplay the nav_view
    :type context: qt_gui.plugin_context.PluginContext
    :param name: The widgets name
    :type name: str
    """
    def __init__(self, context, name='NavView', icon_paths=[]):
        super(NavViewDashWidget, self).__init__(name, icons=[['ic-navigation.svg']], suppress_overlays=True, icon_paths=icon_paths)
        self.context = context

        self._nav_view = None

        self.clicked.connect(self._show_nav_view)

    def _show_nav_view(self):
        if not self._nav_view:
            #TODO: There should be some way to customize the params for nav_view creation
            self._nav_view = NavViewWidget() 
            self._nav_view.destroyed.connect(self._view_closed)

        self.context.add_widget(self._nav_view)

    def _view_closed(self):
        self._nav_view = None
