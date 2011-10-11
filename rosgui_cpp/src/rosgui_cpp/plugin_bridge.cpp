#include <rosgui_cpp/plugin_bridge.h>

#include <rosgui_cpp/plugin.h>
#include <rosgui_cpp/plugin_context.h>
#include <rosgui_cpp/plugin_provider.h>

#include <QEvent>

namespace rosgui_cpp {

PluginBridge::PluginBridge()
  : QObject()
  , provider_(0)
  , plugin_(0)
{
  setObjectName("PluginBridge");
}

bool PluginBridge::load_plugin(PluginProvider* provider, const QString& plugin_id, PluginContext* plugin_context)
{
  provider_ = provider;
  plugin_ = provider_->load_plugin(plugin_id, plugin_context);
  return plugin_ != 0;
}

void PluginBridge::unload_plugin()
{
  provider_->unload_plugin(plugin_);
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
