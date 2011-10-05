#ifndef rosgui_cpp__RecursivePluginProvider_H
#define rosgui_cpp__RecursivePluginProvider_H

#include "composite_plugin_provider.h"
#include "ros_pluginlib_plugin_provider_for_plugin_providers.h"

#include <QMap>
#include <QSet>
#include <QString>

namespace rosgui_cpp
{

class RecursivePluginProvider
  : public CompositePluginProvider
{

public:

  RecursivePluginProvider(RosPluginlibPluginProvider_ForPluginProviders* plugin_provider);

  virtual QMap<QString, QString> discover();

private:

  RosPluginlibPluginProvider_ForPluginProviders* plugin_provider_;

  QSet<PluginProvider*> plugin_providers_;

};

} // namespace

#endif // rosgui_cpp__RecursivePluginProvider_H
