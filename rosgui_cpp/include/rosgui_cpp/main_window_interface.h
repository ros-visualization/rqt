#ifndef rosgui_cpp__MainWindowInterface_H
#define rosgui_cpp__MainWindowInterface_H

#include "generic_proxy.h"
#include "plugin_bridge.h"

#include <QDockWidget>
#include <QWidget>

namespace rosgui_cpp
{

class MainWindowInterface
  : public QWidget
{

public:

  MainWindowInterface(QWidget* obj);

  MainWindowInterface(const MainWindowInterface& other);

  virtual void addDockWidget(Qt::DockWidgetArea area, QDockWidget* dock_widget);

  virtual void set_plugin_instance(PluginBridge* plugin_instance);

protected:

  GenericProxy proxy_;

};

} // namespace

#endif // rosgui_cpp__MainWindowInterface_H
