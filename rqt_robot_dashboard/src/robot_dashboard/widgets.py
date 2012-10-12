"""
.. module:: widgets
    :synopsis: Widgets for the robot_dashboard.

.. moduleauthor:: Ze'ev Klapow <zklapow@willowgarage.com>

This module provides a set of standard widgets for using with the dashboard.

To use them you must provide instances of the to your dashboard in its :func:`get_widgets` method. For example::
    
    from robot_dashboard.dashboard import Dashboard
    from robot_dashboard.widgets import MonitorDashWidget, ConsoleDashWidget

    class MyDashboard(Dashboard):
        def get_widgets(self):
            self.monitor = MonitorDashWidget(self.context)
            self.console = ConsoleDashWidget(self.console)

            return({'Diagnostics': [self.monitor, self.console]})

Would create a simple dashboard with the ability to open a robot_monitor and a ROS console.

Widget Types
============

"""

import roslib;roslib.load_manifest('robot_dashboard')
import rospy
from robot_monitor import RobotMonitor
from nav_view import NavViewWidget

from .util import make_icon

from rqt_console.console_widget import ConsoleWidget
from rqt_console.console_subscriber import ConsoleSubscriber
from rqt_console.message_data_model import MessageDataModel
from rqt_console.message_proxy_model import MessageProxyModel
from diagnostic_msgs.msg import DiagnosticArray

from QtCore import Signal, QMutex, QTimer, QSize
from QtGui import QPushButton, QMenu, QIcon, QWidget, QVBoxLayout, QColor, QProgressBar, QToolButton

from PIL import Image
from PIL.ImageQt import ImageQt

import os.path
import rospkg

rp = rospkg.RosPack()

image_path = os.path.join(rp.get_path('robot_dashboard'), 'images')

class IconToolButton(QToolButton):
    """This is the base class for all widgets. It provides state and icon switching support as well as convenience functions for creating icons.

    .. note:: You must specify either ``icons`` and ``clicked_icons`` or ``icon`` and ``clicked_icon``. Using icon and clicked icon will create a set of icons using the overlays in ``robot_dashboard/images``.

    :param icons: A list of icons for the states of this button.
    :type icons: list
    :param clicked_icons: A list of clicked state icons.
    :type clicked_icons: list
    :param icon: The base icon file path.
    :type icon: str
    :param clicked_icon: The base clicked icon file path.
    :type clicked_icon: str
    """
    state_changed = Signal(int)
    def __init__(self, name, icons = [], clicked_icons = [], icon = '', clicked_icon = ''):
        super(IconToolButton, self).__init__()
        self.name = name
        self.setObjectName(self.name)

        self.state_changed.connect(self._update_state)
        self.pressed.connect(self._pressed)
        self.released.connect(self._released)

        self.setStyleSheet('QToolButton {border: none;}')

        # If the icon argument was specificied, build the default icon states
        if len(icons) == 0 and icon:
            self._icon = self.load_image(icon) 
            self._warn_icon = self.overlay(self._icon, 'warn-overlay.png')
            self._err_icon = self.overlay(self._icon, 'err-overlay.png')
            self._stale_icon = self.overlay(self._icon, 'stale-overlay.png')

            self._icon_click = self.load_image(clicked_icon)
            self._warn_click = self.overlay(self._icon_click, 'warn-overlay.png')
            self._err_click = self.overlay(self._icon_click, 'err-overlay.png')
            self._stale_click = self.overlay(self._icon_click, 'stale-overlay.png')

            icons = [make_icon(self._icon), make_icon(self._warn_icon), make_icon(self._err_icon), make_icon(self._stale_icon)]
            clicked_icons = [make_icon(self._icon_click), make_icon(self._warn_click), make_icon(self._err_click), make_icon(self._stale_click)] 

        # List of QIcons to use for each state
        self._icons = icons
        self._clicked_icons = clicked_icons

        if not len(self._icons) == len(self._clicked_icons):
            rospy.logwarn("%s has a mismatched number of icons and clicked states"%self.name)

        self.state = 0

    def _update_state(self, state):
        if self.isDown():
            self.setIcon(self._clicked_icons[self.state])
        else:
            self.setIcon(self._icons[self.state])

    def update_state(self, state):
        """Set the state of this button. This will also update the icon for the button based on the ``self._icons`` list

        .. note:: States must not be greater than the length of ``self._icons`` unless you implement a custom ``self._update_state``. 

        :param state: The state to set.
        :type state: int
        """
        self.state = state
        self.state_changed.emit(self.state)

    def  _pressed(self):
        self.setIcon(self._clicked_icons[self.state])

    def _released(self):
        self.setIcon(self._icons[self.state])

    def load_image(self, path):
        """Convenience function to help with loading images.
        Path can either be specified as absolute paths or relative to the robot_dashboard/images directory
        
        :param path: The path or name of the image.
        :type path: str
        """
        if os.path.exists(path):
            return Image.open(path)
        elif os.path.exists(os.path.join(image_path, path)):
            return Image.open(os.path.join(image_path, path)) 
        else:
            raise(Exception("Could not load %s"% path))

    def overlay(self, image, name):
        """Convenience function for creating icons with overlays.
        Overlay path can either be specified as absolute paths or relative to the robot_dashboard/images directory.

        :param image: Image to overlay onto.
        :type image: PIL.Image.Image
        :param name: Path of the overlay.
        :type name: str
        """
        over = self.load_image(name)
        return Image.composite(over, image, over) 

class MenuDashWidget(IconToolButton):
    """A widget which displays a pop-up menu when clicked

    :param context: The plugin context to create the widget in.
    :type context: qt_gui.plugin_context.PluginContext
    :param name: The name to give this widget.
    :type name: str
    :param args: A set of actions this menu should perform.
    :type args: QtGui.QAction
    :param icon: The icon to display in this widgets button.
    :type icon: str
    """
    def __init__(self, context, name, *args, **kwargs):
        super(MenuDashWidget, self).__init__(name, icon='mode.png', clicked_icon = 'mode-click.png')
        self.setStyleSheet('QToolButton::menu-indicator {image: url(none.jpg);} QToolButton {border: none;}')
        self.setPopupMode(QToolButton.InstantPopup)
        self.update_state(0)

        self.pressed.disconnect(self._pressed)
        self.released.disconnect(self._released)

        self._menu = QMenu()
        self._menu.aboutToHide.connect(self._released)
        self._menu.aboutToShow.connect(self._pressed)

        for arg in args:
            self._menu.addAction(arg)

        self.setMenu(self._menu)

    def add_separator(self):
        return self._menu.addSeparator()

    def add_action(self, name, callback):
        """Add an action to the menu, and return the newly created action.

        :param name: The name of the action.
        :type name: str
        :param callback: Function to be called when this item is pressed.
        :type callback: callable
        """
        return self._menu.addAction(name, callback)

class MonitorDashWidget(IconToolButton):
    """A widget which brings up the robot_monitor.

    :param context: The plugin context to create the monitor in.
    :type context: qt_gui.plugin_context.PluginContext
    """
    err= Signal()
    warn = Signal()
    def __init__(self, context):
        super(MonitorDashWidget, self).__init__('MonitorWidget', icon='diagnostics.png', clicked_icon = 'diagnostics-click.png')

        self._monitor = None
        self._monitor_sub = rospy.Subscriber('/diagnostics_agg', DiagnosticArray, self._monitor_cb)
        self._last_msg_time = rospy.Time.now()
        self.err.connect(self._error)
        self.warn.connect(self._warning)

        # Only display a state for 10 sec
        self._timer = QTimer()
        self._timer.timeout.connect(self.ok)
        self._timer.setInterval(10000)
        self._timer.start()

        self._last_update = rospy.Time.now()

        self.context = context
        self.clicked.connect(self._show_monitor)

        self.state = 0
        self.update_state(self.state)

        self._monitor_shown = False
        self.setToolTip('Diagnostics')

    def _show_monitor(self):
        if self._monitor is None:
            self._monitor = RobotMonitor('diagnostics_agg')
            self._monitor.destroyed.connect(self._monitor_close)
        try:
            if self._monitor_shown:
                self.context.remove_widget(self._monitor)
            else:
                self.context.add_widget(self._monitor)
        except Exception as e:
            pass
        finally:
            self._monitor_shown = not self._monitor_shown


    def _monitor_cb(self, msg):
        self._last_msg_time = rospy.Time.now()
        warn = 0
        err = 0
        for status in msg.status:
            if status.level == status.WARN:
                warn = warn + 1
            elif status.level == status.ERROR:
                err = err + 1

        if err > 0:
            self.err.emit()
        elif warn > 0:
            self.warn.emit()

    def _error(self):
        self.update_state(2)

        # Restart the timer when a new state arrives
        self._timer.start()

    def _warning(self):
        if self.state <= 1:
            self.update_state(1)

            # Restaert the timer when a new state arrives
            self._timer.start()

    def ok(self):
        # Each time the timer runs we are either ok or stale
        diff = rospy.Time.now() - self._last_msg_time
        if diff.to_sec() > 5:
            self.update_state(3)
        else:
            self.update_state(0)

    def _monitor_close(self):
        if self._monitor:
            self._monitor.close()
            self._monitor = None

    def close(self):
        self._monitor_close()
        self._monitor_sub.unregister()

class ConsoleDashWidget(IconToolButton):
    """A widget which brings up the ROS console.

    :param context: The plugin context to create the monitor in.
    :type context: qt_gui.plugin_context.PluginContext
    """
    def __init__(self, context):
        super(ConsoleDashWidget, self).__init__('ConsoleWidget',[],[],'console.png','console-click.png')

        self._datamodel = MessageDataModel()
        self._proxymodel = MessageProxyModel()
        self._proxymodel.setSourceModel(self._datamodel)

        self._subscriber = ConsoleSubscriber(self._message_cb)

        self._console = None
        self.context = context
        self.clicked.connect(self._show_console)

        self._mutex = QMutex()
        self._timer = QTimer()
        self._timer.timeout.connect(self._insert_messages)
        self._timer.start(100)

        if self._console is None:
            self._console = ConsoleWidget(self._proxymodel, True)
            self._console.destroyed.connect(self._console_destroyed)
        self._console_shown = False
        self.setToolTip("Rosout")
        self.update_state(0)

    def _show_console(self):
        if self._console is None:
            self._console = ConsoleWidget(self._proxymodel, True)
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
    """A Widget which displays incremental battery state, including a status tip.
    To use this widget simply call `update_perc` and `update_time` to change the displayed charge percentage and time remaining, respectively.

    :param context: The plugin context
    :type context: qt_gui.plugin_context.PluginContext
    :param name: The name of this widget
    :type name: str
    """
    def __init__(self, context, name='Battery'):
        super(BatteryDashWidget, self).__init__(name)
        self.setEnabled(False)

        self.setStyleSheet('QToolButton:disabled {}')

        self._icons = [None]
        self._charge_icons = [None]

        for x in range(0, 6):
            self._icons.append(make_icon(self.load_image('battery-%s.png'%(x*20)), 1))
            self._charge_icons.append(make_icon(self.load_image('battery-charge-%s.png'%(x*20)), 1))

        self.charging = False
        self.update_perc(0)

    def update_perc(self, val):
        """Update the displayed battery percentage. The default implementation of this method displays in 20% increments
        :param val: The new value to be displayed.
        :type val: int
        """
        state = round(val/20.0)
        self.update_state(state)

    def _update_state(self, state):
        if self.charging:
            self.setIcon(self._charge_icons[state+1])
        else:
            self.setIcon(self._icons[state+1])

    def update_time(self, val):
        self.time_remaining = val
        self.setStatusTip("%s remaining"%val)

class NavViewDashWidget(IconToolButton):
    """A widget which launches a nav_view widget in order to view and interact with the ROS nav stack
    :param context: The plugin context in which to dsiplay the nav_view
    :type context: qt_gui.plugin_context.PluginContext
    :param name: The widgets name
    :type name: str
    """
    def __init__(self, context, name='NavView'):
        super(NavViewDashWidget, self).__init__(name)
        self.context = context

        self._icon = self.load_image('nav.png')
        self._clicked_icon = self.load_image('nav-click.png')

        self._icons = [make_icon(self._icon)]
        self._clicked_icons = [make_icon(self._clicked_icon)]

        self.setIcon(make_icon(self._icon))

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
