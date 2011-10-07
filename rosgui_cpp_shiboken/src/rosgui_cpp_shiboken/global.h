#undef QT_NO_STL
#undef QT_NO_STL_WCHAR

#ifndef NULL
#define NULL 0
#endif

#include "pyside_global.h"

#include <QtCore/QtCore>
#include <QtGui/QtGui>

#include <rosgui_cpp/composite_plugin_provider.h>
#include <rosgui_cpp/generic_proxy.h>
#include <rosgui_cpp/main_window_interface.h>
#include <rosgui_cpp/plugin.h>
#include <rosgui_cpp/plugin_bridge.h>
#include <rosgui_cpp/plugin_context.h>
#include <rosgui_cpp/plugin_descriptor.h>
#include <rosgui_cpp/plugin_provider.h>
#include <rosgui_cpp/recursive_plugin_provider.h>
#include <rosgui_cpp/ros_pluginlib_plugin_provider_for_plugin_providers.h>
#include <rosgui_cpp/ros_pluginlib_plugin_provider_for_plugins.h>
#include <rosgui_cpp/settings.h>
