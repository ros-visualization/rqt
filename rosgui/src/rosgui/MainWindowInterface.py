import os, webbrowser
from QtBindingHelper import import_from_qt
Signal, Qt, qWarning = import_from_qt(['Signal', 'Qt', 'qWarning'], 'QtCore')
QWidget, QDockWidget = import_from_qt(['QWidget', 'QDockWidget'], 'QtGui')

from rosgui.RosguiDockWidgetTitleBar import RosguiDockWidgetTitleBar

class MainWindowInterface(QWidget):
    load_plugin_signal = Signal(str, int)

    def __init__(self, plugin_manager, main_window, plugin_context, plugin_descriptor, plugin_instance_id):
        QWidget.__init__(self, main_window)
        self.plugin_manager_ = plugin_manager
        self.main_window_ = main_window
        self.plugin_context_ = plugin_context
        self.plugin_descriptor_ = plugin_descriptor
        self.plugin_instance_id_ = plugin_instance_id
        self.plugin_instance_ = None
        self.dock_widgets_ = []
        # force connection type to queued, to delay the 'reloading' giving the 'unloading' time to finish
        self.load_plugin_signal.connect(plugin_manager.load_plugin, type = Qt.QueuedConnection)


    def addDockWidget(self, area, dockwidget):
        # generate unique object name for this dockwidget
        unique_object_name = os.path.join(self.plugin_instance_id_, dockwidget.objectName())
        # find and remove possible remaining dockwidget with this object name
        old_dockwidget = self.main_window_.findChild(QDockWidget, unique_object_name)
        if old_dockwidget is not None:
            qWarning('MainWindowInterface.addDockWidget(): duplicate object name %s, removing old dock widget!')
            self.main_window_.removeDockWidget(old_dockwidget)
        # rename dockwidget object
        dockwidget.setObjectName(unique_object_name)
        # add new dockwidget
        self.main_window_.addDockWidget(area, dockwidget)
        self.dock_widgets_.append(dockwidget)
        self.__updateTitleBar(dockwidget)
        
        
    def show_dockwidgets(self):
        for dockwidget in self.dock_widgets_:
            dockwidget.show()            
        
        
    def set_plugin_instance(self, plugin_instance):
        self.plugin_instance_ = plugin_instance
        for dockwidget in self.dock_widgets_:
            self.__updateTitleBar(dockwidget)            


    def __updateTitleBar(self, dockwidget):
        title_bar = dockwidget.titleBarWidget()
        if title_bar is None:
            title_bar = RosguiDockWidgetTitleBar(dockwidget)
            dockwidget.setTitleBarWidget(title_bar)
            
            # connect extra buttons
            title_bar.connectButton('help', self.help_request)
            title_bar.connectButton('reload', self.reload_request)
        
        # connect settings button to plugin instance
        if hasattr(self.plugin_instance_, 'settings_request'):
            title_bar.connectButton('settings', getattr(self.plugin_instance_, 'settings_request'))
        else:
            title_bar.hideButton('settings')


    def help_request(self):
        plugin_attributes = self.plugin_descriptor_.attributes()
        webbrowser.open_new_tab('http://www.ros.org/wiki/fuerte/Planning/ROS%20GUI/plugins/' + plugin_attributes['library_name'])


    def reload_request(self):
        plugin_id = self.plugin_descriptor_.plugin_id()
        serial_number = self.plugin_context_.serial_number()

        # unload plugin now
        self.plugin_manager_.unload_plugin(self.plugin_instance_id_)
        # deferred call to load_plugin 
        self.load_plugin_signal.emit(plugin_id, serial_number)
