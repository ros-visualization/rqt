#include <rosgui_cpp/plugin_bridge.h>

#include <rosgui_cpp/plugin.h>

#include <QEvent>

namespace rosgui_cpp {

PluginBridge::PluginBridge()
  : QObject()
  , plugin_(0)
{
  setObjectName("PluginBridge");
}

PluginBridge::PluginBridge(Plugin* plugin)
  : QObject()
  , plugin_(plugin)
{
  setObjectName("PluginBridge");

  QObject* obj = this;
  plugin_->setProperty("PluginBridge", qVariantFromValue(obj));
}

void PluginBridge::plugin_defered_delete()
{
  plugin_->setProperty("PluginBridge", QVariant());
  deleteLater();
}

void PluginBridge::close_plugin()
{
  if (plugin_)
  {
    plugin_->closePlugin();
  }
}

void PluginBridge::save_settings(QObject* global_settings, QObject* perspective_settings)
{
  if (plugin_)
  {
    Settings global(global_settings);
    Settings perspective(perspective_settings);
    plugin_->saveSettings(global, perspective);
  }
}

void PluginBridge::restore_settings(QObject* global_settings, QObject* perspective_settings)
{
  if (plugin_)
  {
    Settings global(global_settings);
    Settings perspective(perspective_settings);
    plugin_->restoreSettings(global, perspective);
  }
}

} // namespace
