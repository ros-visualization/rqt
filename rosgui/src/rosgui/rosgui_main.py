#!/usr/bin/env python

# Copyright (c) 2011, Dirk Thomas, Dorian Scholz, TU Darmstadt
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

import os, signal, sys
from optparse import OptionGroup, OptionParser

def rosgui_main():
    parser = OptionParser('usage: %prog [options]')

    parser.add_option('-b', '--qt-binding', dest='qt_binding', type='str', metavar='BINDING',
                      help='choose Qt bindings to be used [pyqt|pyside]')
    parser.add_option('-l', '--lock-perspective', dest='lock_perspective', action="store_true",
                      help='lock the GUI to the used perspective (hide menu bar and close buttons of plugins)')
    parser.add_option('-m', '--multi-process', dest='multi_process', default=False, action="store_true",
                      help='use separate processes for each plugin instance')
    parser.add_option('-p', '--perspective', dest='perspective', type='str', metavar='PERSPECTIVE',
                      help='start with this perspective')
    parser.add_option('-s', '--stand-alone', dest='standalone_plugin', type='str', metavar='PLUGIN',
                      help='start only this plugin (implies -l)')

    group = OptionGroup(parser, 'Options to query information without starting a ROS GUI instance',
                        'These options can be used to query information about valid arguments for various options.')
    group.add_option('--list-perspectives', dest='list_perspectives', action='store_true',
                     help='list available perspectives')
    group.add_option('--list-plugins', dest='list_plugins', action='store_true',
                     help='list available plugins')
    parser.add_option_group(group)

    group = OptionGroup(parser, 'Options to operate on a running ROS GUI instance',
                        'These options can be used to perform actions on a running ROS GUI instance.')
    group.add_option('--command-pid', dest='command_pid', type='int', metavar='PID',
                     help='pid of ROS GUI instance to operate on')
    group.add_option('--command-start-plugin', dest='command_start_plugin', type='str', metavar='PLUGIN',
                     help='start plugin (requires --command-pid)')
    group.add_option('--command-switch-perspective', dest='command_switch_perspective', type='str', metavar='PERSPECTIVE',
                     help='switch perspective (requires --command-pid)')
    parser.add_option_group(group)

    group = OptionGroup(parser, 'Special options for embedding widgets from separate processes',
                        'These options should never be used on the CLI but only from the ROS GUI code itself.')
    group.add_option('--embed-plugin', dest='embed_plugin', type='str', metavar='PLUGIN',
                     help='embed a plugin into an already running ROS GUI instance (requires all other --embed-* options)')
    group.add_option('--embed-plugin-pid', dest='embed_plugin_pid', type='int', metavar='PID',
                     help='pid of ROS GUI instance to embed plugin into (requires all other --embed-* options)')
    group.add_option('--embed-plugin-serial', dest='embed_plugin_serial', type='int', metavar='SERIAL',
                     help='serial number of plugin to be embedded (requires all other --embed-* options)')
    parser.add_option_group(group)

    options, _ = parser.parse_args()

    # check option dependencies
    try:
        list_options = (options.list_perspectives, options.list_plugins)
        list_options_set = [opt for opt in list_options if opt is not None]
        if len(list_options_set) > 1:
            raise RuntimeError, 'Only one --list-* option can be used at a time'

        command_options = (options.command_start_plugin, options.command_switch_perspective)
        command_options_set = [opt for opt in command_options if opt is not None]
        if len(command_options_set) > 1:
            raise RuntimeError, 'Only one --command-* option can be used at a time (except --command-pid which must be given for every command)'
        if len(command_options_set) == 0 and options.command_pid is not None:
            raise RuntimeError, 'Option --command_pid can only be used together with an other -command-* option'

        embed_options = (options.embed_plugin, options.embed_plugin_pid, options.embed_plugin_serial)
        embed_options_set = [opt for opt in embed_options if opt is not None]
        if len(embed_options_set) > 0 and len(embed_options_set) < len(embed_options):
            raise RuntimeError, 'Missing option(s) - all \'--embed-*\' options must be set'

        groups = (list_options_set, command_options_set, embed_options_set)
        groups_set = [opt for opt in groups if len(opt) > 0]
        if len(groups_set) > 1:
            raise RuntimeError, 'Options from different groups (--list, --command, --embed) can not be used together'

    except RuntimeError, e:
        print e
        #parser.parse_args(['--help'])
        # calling --help will exit
        return 1

    # set implicit option dependencies
    if options.standalone_plugin is not None:
        options.lock_perspective = True

    # use qt/glib mainloop integration to get dbus mainloop working
    from dbus.mainloop.glib import DBusGMainLoop
    from dbus import DBusException, Interface, SessionBus
    DBusGMainLoop(set_as_default=True)

    # create application context containing various relevant information
    from ApplicationContext import ApplicationContext
    context = ApplicationContext()
    context.options = options

    base_bus_name = 'org.ros.rosgui'
    # non-special applications provide their pid via dbus
    if len(groups_set) == 0:
        context.dbus_unique_bus_name = base_bus_name + ('.pid%d' % os.getpid())
        from ApplicationDBusInterface import ApplicationDBusInterface
        _dbus_server = ApplicationDBusInterface(base_bus_name)

    # determine host bus name, either based on given pid or via dbus application interface if available
    elif len(command_options_set) > 0 or len(embed_options_set) > 0:
        if options.command_pid is not None:
            context.host_pid = options.command_pid
        elif options.embed_plugin_pid is not None:
            context.host_pid = options.embed_plugin_pid
        else:
            try:
                remote_object = SessionBus().get_object(base_bus_name, '/Application')
            except DBusException:
                pass
            else:
                remote_interface = Interface(remote_object, 'org.ros.rosgui.Application')
                context.host_pid = remote_interface.get_pid()
        if context.host_pid is not None:
            context.dbus_host_bus_name = base_bus_name + ('.pid%d' % context.host_pid)

    # execute command
    if len(command_options_set) > 0:
        if options.command_start_plugin is not None:
            try:
                remote_object = SessionBus().get_object(context.dbus_host_bus_name, '/PluginManager')
            except DBusException:
                if context.host_pid is None:
                    (rc, msg) = (1, 'unable to find ROS GUI instance')
                else:
                    (rc, msg) = (1, 'unable to communicate with ROS GUI instance #%d' % context.host_pid)
            else:
                remote_interface = Interface(remote_object, 'org.ros.rosgui.PluginManager')
                (rc, msg) = remote_interface.start_plugin(options.command_start_plugin)
            if rc == 0:
                print 'rosgui_main() started plugin "%s" in ROS GUI #%d' % (msg, context.host_pid)
            else:
                print 'rosgui_main() could not start plugin "%s" in ROS GUI #%d: %s' % (options.command_start_plugin, context.host_pid, msg)
            return rc
        elif options.command_switch_perspective is not None:
            remote_object = SessionBus().get_object(context.dbus_bus_name, '/PerspectiveManager')
            remote_interface = Interface(remote_object, 'org.ros.rosgui.PerspectiveManager')
            remote_interface.switch_perspective(options.command_switch_perspective)
            print 'rosgui_main() switched to perspective "%s" in ROS GUI #%d' % (options.command_switch_perspective, options.command_pid)
            return 0
        print 'unknown command'
        return 1

    # choose selected or default qt binding
    setattr(sys, 'SELECT_QT_BINDING', options.qt_binding)
    from QtBindingHelper import QT_BINDING

    from QtCore import qDebug, QSettings, Qt, QTimer
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
        #qDebug('rospkg not found - falling back to roslib')
        from RoslibPluginProvider import RoslibPluginProvider
        ActualRosPluginProvider = RoslibPluginProvider

    app = QApplication(sys.argv)
    app.setAttribute(Qt.AA_DontShowIconsInMenus, False)

    settings = QSettings(QSettings.IniFormat, QSettings.UserScope, 'ros.org', 'rosgui')

    if len(embed_options_set) == 0:
        main_window = MainWindow()
        main_window.setDockNestingEnabled(True)
        main_window.statusBar()

        def sigint_handler(*args):
            qDebug('\nsigint_handler()')
            main_window.close()
        signal.signal(signal.SIGINT, sigint_handler)
        # the timer enables triggering the sigint_handler
        timer = QTimer()
        timer.start(500)
        timer.timeout.connect(lambda: None)

        # create own menu bar to share one menu bar on Mac
        menu_bar = QMenuBar()
        if not options.lock_perspective:
            main_window.setMenuBar(menu_bar)

        file_menu = menu_bar.addMenu(menu_bar.tr('File'))
        action = QAction(file_menu.tr('Quit'), file_menu)
        action.setIcon(QIcon.fromTheme('application-exit'))
        action.triggered.connect(main_window.close)
        file_menu.addAction(action)

    else:
        main_window = None
        menu_bar = None

    # setup plugin manager
    plugin_providers = [
        ActualRosPluginProvider('rosgui', 'rosgui_py::Plugin'),
        RecursivePluginProvider(ActualRosPluginProvider('rosgui', 'rosgui_py::PluginProvider')),
    ]
    plugin_provider = CompositePluginProvider(plugin_providers)
    plugin_manager = PluginManager(main_window, plugin_provider, context)

    if options.list_plugins:
        # output available plugins
        print '\n'.join(sorted(plugin_manager.get_plugins().values()))
        return 0

    help_provider = HelpProvider()
    plugin_manager.plugin_help_signal.connect(help_provider.plugin_help_request)

    # setup perspective manager
    perspective_menu = menu_bar.addMenu(menu_bar.tr('Perspectives')) if menu_bar is not None else None
    perspective_manager = PerspectiveManager(settings, perspective_menu, context)

    if options.list_perspectives:
        # output available perspectives
        print '\n'.join(sorted(perspective_manager.perspectives))
        return 0

    # connect various signals and slots
    if main_window is not None:
        # signal changed perspective to update window title
        perspective_manager.perspective_changed_signal.connect(main_window.perspective_changed)
        # signal new settings due to changed perspective
        perspective_manager.save_settings_signal.connect(main_window.save_settings)
        perspective_manager.restore_settings_signal.connect(main_window.restore_settings)

    perspective_manager.save_settings_signal.connect(plugin_manager.save_settings)
    perspective_manager.restore_settings_signal.connect(plugin_manager.restore_settings)

    if main_window is not None:
        # signal before changing plugins to save window state
        plugin_manager.plugins_about_to_change_signal.connect(main_window.save_setup)
        # signal changed plugins to restore window state
        plugin_manager.plugins_changed_signal.connect(main_window.restore_state)
        # signal save settings to store plugin setup on close
        main_window.save_settings_signal.connect(plugin_manager.save_settings)

    if menu_bar is not None:
        about_handler = AboutHandler(main_window)
        help_menu = menu_bar.addMenu(menu_bar.tr('Help'))
        action = QAction(file_menu.tr('About'), help_menu)
        action.setIcon(QIcon.fromTheme('help-about'))
        action.triggered.connect(about_handler.show)
        help_menu.addAction(action)

    if main_window is not None:
        # set initial size - only used without configuration
        main_window.resize(600, 450)

    # load specific plugin
    plugin = None
    plugin_serial = None
    if options.embed_plugin is not None:
        plugin = options.embed_plugin
        plugin_serial = options.embed_plugin_serial
    elif options.standalone_plugin is not None:
        plugin = options.standalone_plugin
        plugin_serial = 0
    if plugin is not None:
        plugins = plugin_manager.find_plugins_by_name(plugin)
        if len(plugins) == 0:
            print 'rosgui_main() found no plugin matching "%s"' % plugin
            return 1
        elif len(plugins) > 1:
            print 'rosgui_main() found multiple plugins matching "%s"\n%s' % (plugin, '\n'.join(plugins.values()))
            return 1
        plugin = plugins.keys()[0]

    qDebug('QtBindingHelper using %s' % QT_BINDING)

    if plugin is not None:
        perspective_manager.set_perspective(plugin, True)
        if not plugin_manager.is_plugin_running(plugin, plugin_serial):
            plugin_manager.load_plugin(plugin, plugin_serial)
    else:
        perspective_manager.set_perspective(options.perspective)

    if main_window is not None:
        main_window.show()

    exit_code = app.exec_()

    # explicitly sync settings to file before exiting
    settings.sync()

    return exit_code


def rosgui_main_filename():
    return os.path.abspath(__file__)


if __name__ == '__main__':
    sys.exit(rosgui_main())
