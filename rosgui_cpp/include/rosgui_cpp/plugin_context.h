#ifndef rosgui_cpp__PluginContext_H
#define rosgui_cpp__PluginContext_H

#include <QMainWindow>
#include <QMap>
#include <QString>
#include <QVariant>

namespace rosgui_cpp
{

class PluginContext
{

public:

  PluginContext(QMainWindow* main_window, const QString& serial_number);

  QMainWindow* main_window();

  QString serial_number();

  const QMap<QString, QVariant>& attributes() const;

  QVariant attribute(const QString& key) const;

  void set_attribute(const QString& key, const QVariant& value);

protected:

  QMainWindow* main_window_;

  QString serial_number_;

  QMap<QString, QVariant> attributes_;

};

} // namespace

#endif // rosgui_cpp__PluginContext_H
