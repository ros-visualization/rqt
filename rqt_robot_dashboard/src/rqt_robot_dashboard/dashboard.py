import roslib;roslib.load_manifest('rqt_robot_dashboard')
import rospy

from python_qt_binding import loadUi

from python_qt_binding.QtCore import QSize
from python_qt_binding.QtGui import QWidget, QHBoxLayout, QGroupBox, QToolBar
from qt_gui.plugin import Plugin

class Dashboard(Plugin):
    """Base class from which dashboards should inherit."""
    def __init__(self, context, name = None):
        super(Dashboard, self).__init__(context)
        self.context = context
        
        self.setup(context)

        if not hasattr(self, 'name'):
            if not name:
                self.name = 'Dashboard'
            else:
                self.name = name

        self._main_widget = QToolBar()
        self._main_widget.setIconSize(QSize(80, 80))
        self._main_widget.setObjectName(self.name)
        self._main_widget.setWindowTitle(self.name)
        widgets = self.get_widgets()

        layout = QHBoxLayout()

        self._widgets = []
        for v in widgets:
            for i in v:
                try:
                    self._main_widget.addWidget(i)
                    self._widgets.append(i)
                except:
                    raise(Exception("All widgets must be a subclass of QWidget!"))

            self._main_widget.addSeparator()

        # Display the dashboard
        context.add_toolbar(self._main_widget)

    def setup(self, context):
        """Called during ``__init__`` Subclasses should do initialization here.
        
        .. note::
            If this method is overriden it is important to call ``self.setObjectName()`` so that object names do not conflict.

        :param context: The plugin context
        :type context: qt_gui.plugin.Plugin
        """
        pass

    def shutdown_plugin(self):
        """Called when the toolbar is closed by Qt.
        """
        for widget in self._widgets:
            if hasattr(widget, 'close'):
                widget.close()

        self.shutdown_dashboard()

    def shutdown_dashboard(self):
        """Called after shutdown plugin, subclasses should do cleanup here, not in shutdown_plugin
        """
        pass

    def get_widgets(self):
        """
        Most of the dashboard customization should be done here. If this function is not overriden the dashboard will display nothing.

        :returns: List of lists containing dashboard widgets.
        """
        return []
