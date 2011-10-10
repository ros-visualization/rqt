#!/usr/bin/env python

import sys
from optparse import OptionParser

import QtBindingHelper #@UnusedImport
from QtCore import qDebug, QSettings, qWarning
from QtGui import QAction, QApplication, QIcon, QMenuBar

from AboutHandler import AboutHandler
from CompositePluginProvider import CompositePluginProvider
from HelpProvider import HelpProvider
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

    # parse command line
    usage = 'usage: %prog [options]'
    parser = OptionParser(usage)

    parser.add_option('-p', '--perspective', dest='perspective', default=None, type='str',
                    help='startup with this perspective')

    parser.add_option('-s', '--stand-alone', dest='standalone_plugin', default=None, type='str',
                    help='start only this plugin (implies -l)')

    parser.add_option('-l', '--lock-perspective', dest='lock_perspective', action="store_true",
                    help='locks the GUI to the used perspective (hides menu bar and dock widget close buttons)')

    options, _ = parser.parse_args()
    if options.standalone_plugin is not None:
        options.lock_perspective = True

    app = QApplication(sys.argv)
    app.lastWindowClosed.connect(app.quit)

    settings = QSettings(QSettings.IniFormat, QSettings.UserScope, 'ros.org', 'rosgui')

    main_window = MainWindow()
    main_window.setDockNestingEnabled(True)
    main_window.statusBar()

    # create own menu bar to share one menu bar on Mac
    menu_bar = QMenuBar()
    if not options.lock_perspective:
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
    plugin_manager = PluginManager(main_window, plugin_menu, running_menu, plugin_provider, options.lock_perspective)

    help_provider = HelpProvider()
    plugin_manager.plugin_help_signal.connect(help_provider.plugin_help_request)

    perspective_menu = menu_bar.addMenu(menu_bar.tr('Perspectives'))
    perspective_manager = PerspectiveManager(settings, perspective_menu)

    # signal changed perspective to update window title
    perspective_manager.perspective_changed_signal.connect(main_window.perspective_changed)
    # signal new settings due to changed perspective
    perspective_manager.save_settings_signal.connect(main_window.save_settings)
    perspective_manager.restore_settings_signal.connect(main_window.restore_settings)
    perspective_manager.save_settings_signal.connect(plugin_manager.save_settings)
    perspective_manager.restore_settings_signal.connect(plugin_manager.restore_settings)
    # signal before changing plugins to save window state
    plugin_manager.plugins_about_to_change_signal.connect(main_window.save_setup)
    # signal changed plugins to restore window state
    plugin_manager.plugins_changed_signal.connect(main_window.restore_state)
    # signal save settings to store plugin setup on close
    main_window.save_settings_signal.connect(plugin_manager.save_settings)

    about_handler = AboutHandler(main_window)
    help_menu = menu_bar.addMenu(menu_bar.tr('Help'))
    action = QAction(file_menu.tr('About'), help_menu)
    action.setIcon(QIcon.fromTheme('help-about'))
    action.setIconVisibleInMenu(True)
    action.triggered.connect(about_handler.show)
    help_menu.addAction(action)

    # set initial size - only used without configuration
    main_window.resize(600, 450)

    if options.standalone_plugin is not None:
        found_plugins = plugin_manager.find_plugin_by_name(options.standalone_plugin)
        if len(found_plugins) == 0:
            qWarning('rosgui_main(): found no plugin matching "%s"' % options.standalone_plugin)
            return 1

        elif len(found_plugins) > 1:
            qWarning('rosgui_main(): found multiple plugins matching "%s"\n%s' % (options.standalone_plugin, '\n'.join(found_plugins.values())))
            return 1

        perspective_manager.set_perspective(found_plugins.values()[0], True)
        plugin_manager.load_plugin(found_plugins.keys()[0], 0)

    else:
        perspective_manager.set_perspective(options.perspective)

    main_window.show()

    return app.exec_()


if __name__ == '__main__':
    sys.exit(rosgui_main())
