#undef QT_NO_STL
#undef QT_NO_STL_WCHAR

#ifndef NULL
#define NULL 0
#endif

#include "pyside_global.h"

#include <QtCore/QtCore>
#include <QtGui/QtGui>

#include <qt_gui_cpp/composite_plugin_provider.h>
#include <qt_gui_cpp/generic_proxy.h>
#include <qt_gui_cpp/plugin.h>
#include <qt_gui_cpp/plugin_bridge.h>
#include <qt_gui_cpp/plugin_context.h>
#include <qt_gui_cpp/plugin_descriptor.h>
#include <qt_gui_cpp/plugin_provider.h>
#include <qt_gui_cpp/recursive_plugin_provider.h>
#include <qt_gui_cpp/ros_pluginlib_plugin_provider.h>
#include <qt_gui_cpp/ros_pluginlib_plugin_provider_for_plugin_providers.h>
#include <qt_gui_cpp/ros_pluginlib_plugin_provider_for_plugins.h>
#include <qt_gui_cpp/settings.h>
