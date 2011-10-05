#include <rosgui_cpp/plugin_context.h>

namespace rosgui_cpp {

PluginContext::PluginContext(QMainWindow* main_window, const QString& serial_number)
  : main_window_(main_window)
  , serial_number_(serial_number)
{}

QMainWindow* PluginContext::main_window()
{
  return main_window_;
}

QString PluginContext::serial_number()
{
  return serial_number_;
}

const QMap<QString, QVariant>& PluginContext::attributes() const
{
  return attributes_;
}

QVariant PluginContext::attribute(const QString& key) const
{
  if (attributes_.contains(key))
  {
    return attributes_[key];
  }
  return QVariant();
}

void PluginContext::set_attribute(const QString& key, const QVariant& value)
{
  attributes_[key] = value;
}

} // namespace
