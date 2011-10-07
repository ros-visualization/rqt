#include <rosgui_cpp/main_window_interface.h>

#include <stdexcept>

namespace rosgui_cpp {

MainWindowInterface* MainWindowInterface::create_instance(QWidget* obj)
{
  return new MainWindowInterface(obj);
}

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
  QGenericArgument arg("QDockWidget", dock_widget);
  bool rc = proxy_.invokeMethod("addDockWidget", Q_ARG(int, area), arg);
  if (!rc) throw std::runtime_error("MainWindowInterface::addDockWidget() invoke method failed");
}

void MainWindowInterface::set_plugin_instance(PluginBridge* plugin_instance)
{
  QGenericArgument arg("QObject", plugin_instance);
  bool rc = proxy_.invokeMethod("set_plugin_instance",arg);
  if (!rc) throw std::runtime_error("MainWindowInterface::set_plugin_instance() invoke method failed");
}

} // namespace
