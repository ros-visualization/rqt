namespace rosgui_cpp
{

class PluginBridge
  : QObject
{
%TypeHeaderCode
#include <rosgui_cpp/plugin_bridge.h>
%End

public:

  PluginBridge();

  virtual bool load_plugin(rosgui_cpp::PluginProvider* provider, const QString& plugin_id, rosgui_cpp::PluginContext* plugin_context);

  virtual void unload_plugin();

  virtual void plugin_defered_delete();

public slots:

  virtual void close_plugin();

  virtual void save_settings(QObject* global_settings, QObject* perspective_settings);

  virtual void restore_settings(QObject* global_settings, QObject* perspective_settings);

};

};
