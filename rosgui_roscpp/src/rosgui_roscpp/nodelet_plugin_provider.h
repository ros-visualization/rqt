#ifndef rosgui_roscpp__NodeletPluginProvider_H
#define rosgui_roscpp__NodeletPluginProvider_H

#include <rosgui_cpp/ros_pluginlib_plugin_provider.h>

#include <rosgui_roscpp/plugin.h>

#include <string>

namespace rosgui_roscpp {

class RosCppPluginProvider;

class NodeletPluginProvider
  : public rosgui_cpp::RosPluginlibPluginProvider<rosgui_roscpp::Plugin>
{

public:

  NodeletPluginProvider(const QString& export_tag, const QString& base_class_type, RosCppPluginProvider* manager);

protected:

  virtual void init_plugin(const QString& plugin_id, rosgui_cpp::PluginContext* plugin_context, rosgui_cpp::Plugin* plugin);

  RosCppPluginProvider* manager_;

};

}

#endif // rosgui_roscpp__NodeletPluginProvider_H
