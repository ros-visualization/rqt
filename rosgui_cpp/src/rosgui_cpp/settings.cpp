#include <rosgui_cpp/settings.h>

#include <stdexcept>

namespace rosgui_cpp {

Settings::Settings(QObject* obj)
  : proxy_(obj)
{}

Settings Settings::getSettings(const QString& prefix)
{
  Settings settings(proxy_.proxiedObject());
  bool rc = proxy_.invokeMethodWithReturn("get_settings", Q_RETURN_ARG(Settings, settings), Q_ARG(QString, prefix));
  if (!rc) throw std::runtime_error("Settings::get_settings() invoke method failed");
  return settings;
}

QStringList Settings::allKeys() const
{
  QStringList list;
  bool rc = const_cast<Settings*>(this)->proxy_.invokeMethodWithReturn("all_keys", Q_RETURN_ARG(QStringList, list));
  if (!rc) throw std::runtime_error("Settings::all_keys() invoke method failed");
  return list;
}

QStringList Settings::childGroups() const
{
  QStringList list;
  bool rc = const_cast<Settings*>(this)->proxy_.invokeMethodWithReturn("child_groups", Q_RETURN_ARG(QStringList, list));
  if (!rc) throw std::runtime_error("Settings::child_groups() invoke method failed");
  return list;
}

QStringList Settings::childKeys() const
{
  QStringList list;
  bool rc = const_cast<Settings*>(this)->proxy_.invokeMethodWithReturn("child_keys", Q_RETURN_ARG(QStringList, list));
  if (!rc) throw std::runtime_error("Settings::child_keys() invoke method failed");
  return list;
}

bool Settings::contains(const QString& key) const
{
  bool flag = false;
  bool rc = const_cast<Settings*>(this)->proxy_.invokeMethodWithReturn("contains", Q_RETURN_ARG(bool, flag), Q_ARG(QString, key));
  if (!rc) throw std::runtime_error("Settings::contains() invoke method failed");
  return flag;
}

void Settings::remove(const QString& key)
{
  bool rc = proxy_.invokeMethod("remove", Q_ARG(QString, key));
  if (!rc) throw std::runtime_error("Settings::remove() invoke method failed");
}

void Settings::setValue(const QString& key, const QVariant& value)
{
  bool rc = proxy_.invokeMethod("set_value", Q_ARG(QString, key), Q_ARG(QVariant, value));
  if (!rc) throw std::runtime_error("Settings::set_value() invoke method failed");
}

QVariant Settings::value(const QString& key, const QVariant& defaultValue) const
{
  QVariant val;
  bool rc = const_cast<Settings*>(this)->proxy_.invokeMethodWithReturn("value", Q_RETURN_ARG(QVariant, val), Q_ARG(QString, key), Q_ARG(QVariant, defaultValue));
  if (!rc) throw std::runtime_error("Settings::value() invoke method failed");
  return val;
}

} // namespace
