#include <rosgui_cpp/composite_plugin_provider.h>

#include <stdexcept>

namespace rosgui_cpp {

CompositePluginProvider::CompositePluginProvider(const QList<PluginProvider*>& plugin_providers)
  : PluginProvider()
  , plugin_providers_(plugin_providers)
{}

void CompositePluginProvider::set_plugin_providers(const QList<PluginProvider*>& plugin_providers)
{
  plugin_providers_ = plugin_providers;
}

QList<PluginDescriptor*> CompositePluginProvider::discover_descriptors()
{
  // discover plugins from all providers
  QList<PluginDescriptor*> descriptors;
  for (QList<PluginProvider*>::iterator it = plugin_providers_.begin(); it != plugin_providers_.end(); it++)
  {
    QList<PluginDescriptor*> sub_descriptors;
    try
    {
      sub_descriptors = (*it)->discover_descriptors();
    }
    catch (std::runtime_error e)
    {
      qCritical("CompositePluginProvider::discover() could not discover plugins from provider - runtime_error");
      return descriptors;
    }
    catch (...)
    {
      qCritical("CompositePluginProvider::discover() could not discover plugins from provider");
      return descriptors;
    }

    QSet<QString> plugin_ids;
    for (QList<PluginDescriptor*>::iterator jt = sub_descriptors.begin(); jt != sub_descriptors.end(); jt++)
    {
      PluginDescriptor* descriptor = *jt;
      descriptors.append(descriptor);
      plugin_ids.insert(descriptor->pluginId());
    }
    discovered_plugins_[*it] = plugin_ids;
  }
  return descriptors;
}

void* CompositePluginProvider::load(const QString& plugin_id, PluginContext* plugin_context)
{
  // dispatch load to appropriate provider
  for (QMap<PluginProvider*, QSet<QString> >::iterator it = discovered_plugins_.begin(); it != discovered_plugins_.end(); it++)
  {
    if (it.value().contains(plugin_id))
    {
      PluginProvider* plugin_provider = it.key();
      void* instance = plugin_provider->load(plugin_id, plugin_context);
      running_plugins_[instance] = plugin_provider;
      return instance;
    }
  }
  return 0;
}

Plugin* CompositePluginProvider::load_plugin(const QString& plugin_id, PluginContext* plugin_context)
{
  // dispatch load to appropriate provider
  for (QMap<PluginProvider*, QSet<QString> >::iterator it = discovered_plugins_.begin(); it != discovered_plugins_.end(); it++)
  {
    if (it.value().contains(plugin_id))
    {
      PluginProvider* plugin_provider = it.key();
      Plugin* instance = plugin_provider->load_plugin(plugin_id, plugin_context);
      running_plugins_[instance] = plugin_provider;
      return instance;
    }
  }
  return 0;
}

void CompositePluginProvider::unload(void* plugin_instance)
{
  // dispatch unload to appropriate provider
  QMap<void*, PluginProvider*>::iterator it = running_plugins_.find(plugin_instance);
  if (it != running_plugins_.end())
  {
    (*it)->unload(plugin_instance);
    running_plugins_.remove(it);
    return;
  }
  throw std::runtime_error("plugin_instance not found");
}

} // namespace
