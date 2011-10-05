#include <rosgui_cpp/recursive_plugin_provider.h>

#include <stdexcept>

namespace rosgui_cpp {

RecursivePluginProvider::RecursivePluginProvider(RosPluginlibPluginProvider_ForPluginProviders* plugin_provider)
  : CompositePluginProvider()
  , plugin_provider_(plugin_provider)
{}

QMap<QString, QString> RecursivePluginProvider::discover()
{
  // discover plugins, which are providers themselves
  QList<PluginDescriptor*> descriptors = plugin_provider_->discover_descriptors();
  QList<QString> plugin_ids;
  for (QList<PluginDescriptor*>::iterator it = descriptors.begin(); it != descriptors.end(); it++)
  {
    PluginDescriptor* descriptor = *it;
    plugin_ids.append(descriptor->pluginId());
    delete descriptor;
  }

  // instantiate plugins
  QList<PluginProvider*> providers;
  for (QList<QString>::iterator it = plugin_ids.begin(); it != plugin_ids.end(); it++)
  {
    try
    {
      // pass NULL as PluginContext for PluginProviders
      PluginProvider* instance = plugin_provider_->load_explicit_type(*it, 0);
      if (instance == 0)
      {
        throw std::runtime_error("load returned None");
      }
      providers.append(instance);
    }
    catch (...)
    {
      qCritical("RecursivePluginProvider.discover() loading plugin '%s' failed", it->toStdString().c_str());
    }
  }

  // delegate discovery through instantiated plugin providers to base class
  set_plugin_providers(providers);
  return CompositePluginProvider::discover();
}

} // namespace
