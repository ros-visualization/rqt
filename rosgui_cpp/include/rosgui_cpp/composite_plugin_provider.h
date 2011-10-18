#ifndef rosgui_cpp__CompositePluginProvider_H
#define rosgui_cpp__CompositePluginProvider_H

#include "plugin_descriptor.h"
#include "plugin_provider.h"

#include <QList>
#include <QMap>
#include <QSet>
#include <QString>

namespace rosgui_cpp
{

class CompositePluginProvider
  : public PluginProvider
{

public:

  CompositePluginProvider(const QList<PluginProvider*>& plugin_providers = QList<PluginProvider*>());

  virtual ~CompositePluginProvider();

  virtual void set_plugin_providers(const QList<PluginProvider*>& plugin_providers);

  virtual QList<PluginDescriptor*> discover_descriptors();

  virtual void* load(const QString& plugin_id, PluginContext* plugin_context);

  virtual Plugin* load_plugin(const QString& plugin_id, PluginContext* plugin_context);

  virtual void unload(void* plugin_instance);

private:

  QList<PluginProvider*> plugin_providers_;

  QMap<PluginProvider*, QSet<QString> > discovered_plugins_;

  QMap<void*, PluginProvider*> running_plugins_;

};

} // namespace

#endif // rosgui_cpp__CompositePluginProvider_H
