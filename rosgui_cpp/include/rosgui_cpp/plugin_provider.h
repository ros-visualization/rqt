#ifndef rosgui_cpp__PluginProvider_H
#define rosgui_cpp__PluginProvider_H

#include "plugin.h"
#include "plugin_context.h"
#include "plugin_descriptor.h"

#include <QList>
#include <QMap>
#include <QString>

namespace rosgui_cpp
{

class PluginProvider
{

public:

  PluginProvider();

  virtual ~PluginProvider();

  virtual QMap<QString, QString> discover();

  /**
   * @attention Ownership of returned PluginDescriptor's is transfered to the caller
   */
  virtual QList<PluginDescriptor*> discover_descriptors();

  virtual void* load(const QString& plugin_id, PluginContext* plugin_context);

  virtual Plugin* load_plugin(const QString& plugin_id, PluginContext* plugin_context);

  virtual void unload(void* plugin_instance);

  virtual void unload_plugin(Plugin* plugin_instance);

};

} // namespace

#endif // rosgui_cpp__PluginProvider_H
