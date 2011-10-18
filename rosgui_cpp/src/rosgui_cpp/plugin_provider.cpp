#include <rosgui_cpp/plugin_provider.h>

namespace rosgui_cpp {

PluginProvider::PluginProvider()
{}

PluginProvider::~PluginProvider()
{}

QMap<QString, QString> PluginProvider::discover()
{
  QMap<QString, QString> plugins;
  QList<PluginDescriptor*> descriptors = discover_descriptors();
  for (QList<PluginDescriptor*>::iterator it = descriptors.begin(); it != descriptors.end(); it++)
  {
    // extract plugin descriptor dictionary
    PluginDescriptor* descriptor = *it;
    QMap<QString, QString> plugin = descriptor->toDictionary();
    plugins.unite(plugin);
    delete descriptor;
  }
  return plugins;
}

QList<PluginDescriptor*> PluginProvider::discover_descriptors()
{
  return QList<PluginDescriptor*>();
}

void* PluginProvider::load(const QString& plugin_id, PluginContext* plugin_context)
{
  return load_plugin(plugin_id, plugin_context);
}

Plugin* PluginProvider::load_plugin(const QString& plugin_id, PluginContext* plugin_context)
{
  return 0;
}

void PluginProvider::unload(void* plugin_instance)
{}

void PluginProvider::unload_plugin(Plugin* plugin_instance)
{
  unload(plugin_instance);
}

} // namespace
