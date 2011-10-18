#include <rosgui_cpp/main_window_interface.h>

#include <stdexcept>

namespace rosgui_cpp {

MainWindowInterface::MainWindowInterface(QWidget* obj)
  : QWidget(obj)
  , proxy_(obj)
{}

MainWindowInterface::MainWindowInterface(const MainWindowInterface& other)
  : QWidget(other.parentWidget())
  , proxy_(other.parentWidget())
{}

void MainWindowInterface::addDockWidget(Qt::DockWidgetArea area, QDockWidget* dock_widget)
{
  bool rc = proxy_.invokeMethod("addDockWidget", Q_ARG(int, area), Q_ARG(QDockWidget*, dock_widget));
  if (!rc) throw std::runtime_error("MainWindowInterface::addDockWidget() invoke method failed");
}

void MainWindowInterface::removeDockWidget(QDockWidget* dock_widget)
{
  bool rc = proxy_.invokeMethod("removeDockWidget", Q_ARG(QDockWidget*, dock_widget));
  if (!rc) throw std::runtime_error("MainWindowInterface::removeDockWidget() invoke method failed");
}

void MainWindowInterface::set_plugin_instance(PluginBridge* plugin_instance)
{
  bool rc = proxy_.invokeMethod("set_plugin_instance", Q_ARG(QObject*, plugin_instance));
  if (!rc) throw std::runtime_error("MainWindowInterface::set_plugin_instance() invoke method failed");
}

} // namespace
