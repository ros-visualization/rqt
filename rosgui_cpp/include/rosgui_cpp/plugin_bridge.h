#ifndef rosgui_cpp__PluginBridge_H
#define rosgui_cpp__PluginBridge_H

#include <QObject>

namespace rosgui_cpp
{

class Plugin;

class PluginBridge
  : public QObject
{

  Q_OBJECT

public:

  PluginBridge();

  PluginBridge(Plugin* plugin);

  virtual void plugin_defered_delete();

public slots:

  virtual void close_plugin();

  virtual void save_settings(QObject* global_settings, QObject* perspective_settings);

  virtual void restore_settings(QObject* global_settings, QObject* perspective_settings);

private:

  Plugin* plugin_;

};

} // namespace

#endif // rosgui_cpp__PluginBridge_H
