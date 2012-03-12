namespace rosgui_cpp
{

class RosPluginlibPluginProvider_ForPluginProviders
  : QObject
  , rosgui_cpp::PluginProvider
{
%TypeHeaderCode
#include <rosgui_cpp/ros_pluginlib_plugin_provider_for_plugin_providers.h>
%End

public:

  static rosgui_cpp::RosPluginlibPluginProvider_ForPluginProviders* create_instance(const QString& export_tag, const QString& base_class_type);

  RosPluginlibPluginProvider_ForPluginProviders(const QString& export_tag, const QString& base_class_type);

  virtual ~RosPluginlibPluginProvider_ForPluginProviders();

  virtual QMap<QString, QString> discover();

  virtual QList<rosgui_cpp::PluginDescriptor*> discover_descriptors();

  virtual void* load(const QString& plugin_id, rosgui_cpp::PluginContext* plugin_context);

  virtual rosgui_cpp::Plugin* load_plugin(const QString& plugin_id, rosgui_cpp::PluginContext* plugin_context);

  virtual rosgui_cpp::PluginProvider* load_explicit_type(const QString& plugin_id, rosgui_cpp::PluginContext* plugin_context);

  virtual void unload(void* instance);

};

};
