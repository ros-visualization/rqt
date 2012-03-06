namespace rosgui_cpp
{

class PluginProvider
{
%TypeHeaderCode
#include <rosgui_cpp/plugin_provider.h>
%End

public:

  PluginProvider();

  virtual ~PluginProvider();

  virtual QMap<QString, QString> discover();

  /**
   * @attention Ownership of returned PluginDescriptor's is transfered to the caller
   */
  virtual QList<rosgui_cpp::PluginDescriptor*> discover_descriptors();

  virtual void* load(const QString& plugin_id, rosgui_cpp::PluginContext* plugin_context);

  virtual rosgui_cpp::Plugin* load_plugin(const QString& plugin_id, rosgui_cpp::PluginContext* plugin_context);

  virtual void unload(void* plugin_instance);

  virtual void unload_plugin(rosgui_cpp::Plugin* plugin_instance);

};

};
