namespace rosgui_cpp
{

class CompositePluginProvider
  : rosgui_cpp::PluginProvider
{
%TypeHeaderCode
#include <rosgui_cpp/composite_plugin_provider.h>
%End

public:

  CompositePluginProvider(const QList<rosgui_cpp::PluginProvider*>& plugin_providers = QList<rosgui_cpp::PluginProvider*>());

  virtual void set_plugin_providers(const QList<rosgui_cpp::PluginProvider*>& plugin_providers);

  virtual QList<rosgui_cpp::PluginDescriptor*> discover_descriptors();

  virtual void* load(const QString& plugin_id, rosgui_cpp::PluginContext* plugin_context);

  virtual rosgui_cpp::Plugin* load_plugin(const QString& plugin_id, rosgui_cpp::PluginContext* plugin_context);

  virtual void unload(void* plugin_instance);

};

};
