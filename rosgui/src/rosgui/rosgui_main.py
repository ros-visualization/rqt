#!/usr/bin/env python

import sys

from QtBindingHelper import import_from_qt
QSettings, qDebug = import_from_qt(['QSettings', 'qDebug'], 'QtCore')
QAction, QApplication, QIcon, QMenuBar = import_from_qt(['QAction', 'QApplication', 'QIcon', 'QMenuBar'], 'QtGui')

from CompositePluginProvider import CompositePluginProvider
from MainWindow import MainWindow
from PerspectiveManager import PerspectiveManager
from PluginManager import PluginManager
from RecursivePluginProvider import RecursivePluginProvider

try:
    from RospkgPluginProvider import RospkgPluginProvider
    ActualRosPluginProvider = RospkgPluginProvider
except ImportError:
    qDebug('rospkg not found - falling back to roslib')
    from RoslibPluginProvider import RoslibPluginProvider
    ActualRosPluginProvider = RoslibPluginProvider

def rosgui_main():
    qDebug('rosgui_main()')
    app = QApplication(sys.argv)
    app.lastWindowClosed.connect(app.quit)

    settings = QSettings(QSettings.IniFormat, QSettings.UserScope, 'ros.org', 'rosgui')

    main_window = MainWindow()
    main_window.setDockNestingEnabled(True)
    main_window.statusBar()

    # create own menu bar to share one menu bar on Mac
    menu_bar = QMenuBar()
    main_window.setMenuBar(menu_bar)

    file_menu = menu_bar.addMenu(menu_bar.tr('File'))
    action = QAction(file_menu.tr('Quit'), file_menu)
    action.setIcon(QIcon.fromTheme('application-exit'))
    action.setIconVisibleInMenu(True)
    action.triggered.connect(main_window.close)
    file_menu.addAction(action)

    plugin_menu = menu_bar.addMenu(menu_bar.tr('Plugins'))
    running_menu = menu_bar.addMenu(menu_bar.tr('Running'))
    plugin_providers = [
        ActualRosPluginProvider('rosgui', 'rosgui_py::Plugin'),
        RecursivePluginProvider(ActualRosPluginProvider('rosgui', 'rosgui_py::PluginProvider')),
    ]
    plugin_provider = CompositePluginProvider(plugin_providers)
    plugin_manager = PluginManager(main_window, plugin_menu, running_menu, plugin_provider)

    perspective_menu = menu_bar.addMenu(menu_bar.tr('Perspectives'))
    perspective_manager = PerspectiveManager(settings, perspective_menu)

    # signal changed perspective to update window title
    perspective_manager.perspective_changed_signal.connect(main_window.perspective_changed)
    # signal new settings due to changed perspective
    perspective_manager.settings_changed_signal.connect(main_window.settings_changed)
    perspective_manager.settings_changed_signal.connect(plugin_manager.settings_changed)
    # signal before changing plugins to save window state
    plugin_manager.plugins_about_to_change_signal.connect(main_window.save_setup)
    # signal changed plugins to restore window state
    plugin_manager.plugins_changed_signal.connect(main_window.restore_setup)
    # signal save settings to store plugin setup on close
    main_window.save_settings_signal.connect(plugin_manager.save_settings)

    perspective = None
    for arg in sys.argv:
        if arg.startswith('--perspective='):
            perspective = arg.split('=', 1)[1]

    perspective_manager.set_perspective(perspective)

    main_window.show()

    return app.exec_()


if __name__ == '__main__':
    sys.exit(rosgui_main())
