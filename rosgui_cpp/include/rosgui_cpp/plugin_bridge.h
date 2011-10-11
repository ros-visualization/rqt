#ifndef rosgui_cpp__PluginBridge_H
#define rosgui_cpp__PluginBridge_H

#include <QObject>

namespace rosgui_cpp
{

class Plugin;
class PluginContext;
class PluginProvider;

class PluginBridge
  : public QObject
{

  Q_OBJECT

public:

  PluginBridge();

  virtual bool load_plugin(PluginProvider* provider, const QString& plugin_id, PluginContext* plugin_context);

  virtual void unload_plugin();

  virtual void plugin_defered_delete();

public slots:

  virtual void close_plugin();

  virtual void save_settings(QObject* global_settings, QObject* perspective_settings);

  virtual void restore_settings(QObject* global_settings, QObject* perspective_settings);

private:

  PluginProvider* provider_;

  Plugin* plugin_;

};

} // namespace

#endif // rosgui_cpp__PluginBridge_H
