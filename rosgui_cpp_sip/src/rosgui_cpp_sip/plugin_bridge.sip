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

  PluginBridge(rosgui_cpp::Plugin* plugin);

  virtual void plugin_defered_delete();

public slots:

  virtual void close_plugin();

  virtual void save_settings(QObject* global_settings, QObject* perspective_settings);

  virtual void restore_settings(QObject* global_settings, QObject* perspective_settings);

};

};
