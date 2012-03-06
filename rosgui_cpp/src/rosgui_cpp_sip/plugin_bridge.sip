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

  virtual bool has_configuration() const;

  virtual void trigger_configuration();

public slots:

  virtual void shutdown_plugin();

  virtual void save_settings(QObject* global_settings, QObject* perspective_settings);

  virtual void restore_settings(QObject* global_settings, QObject* perspective_settings);

};

};
