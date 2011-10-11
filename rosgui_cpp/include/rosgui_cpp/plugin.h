#ifndef rosgui_cpp__Plugin_H
#define rosgui_cpp__Plugin_H

#include "plugin_bridge.h"
#include "plugin_context.h"
#include "settings.h"

#include <QObject>

namespace rosgui_cpp
{

class Plugin
  : public QObject
{

public:

  Plugin()
    : QObject()
  {}

  virtual void initPlugin(PluginContext& /*context*/)
  {}

  virtual void closePlugin()
  {}

  virtual void saveSettings(Settings& /*global_settings*/, Settings& /*perspective_settings*/)
  {}

  virtual void restoreSettings(Settings& /*global_settings*/, Settings& /*perspective_settings*/)
  {}

protected:

  virtual void deletePluginLater()
  {
    QVariant p = property("PluginBridge");
    if (p.isValid())
    {
      QObject* obj = qVariantValue<QObject*>(p);
      if (!obj)
      {
        qWarning("Plugin::delete_plugin_later() property not a QObject");
      }
      else
      {
        PluginBridge* bridge = qobject_cast<PluginBridge*>(obj);
        if (!bridge)
        {
          qWarning("Plugin::delete_plugin_later() property not a PluginBridge");
        }
        else
        {
          bridge->plugin_defered_delete();
        }
      }
    }
  }

};

} // namespace

#endif // rosgui_cpp__Plugin_H
