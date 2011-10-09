#ifndef rosgui_cpp__PluginContext_H
#define rosgui_cpp__PluginContext_H

#include "main_window_interface.h"

#include <QMap>
#include <QString>
#include <QVariant>

namespace rosgui_cpp
{

class PluginContext
{

public:

  PluginContext(MainWindowInterface* main_window, int serial_number);

  MainWindowInterface* main_window();

  int serial_number();

  const QMap<QString, QVariant>& attributes() const;

  QVariant attribute(const QString& key) const;

  void set_attribute(const QString& key, const QVariant& value);

protected:

  MainWindowInterface* main_window_;

  int serial_number_;

  QMap<QString, QVariant> attributes_;

};

} // namespace

#endif // rosgui_cpp__PluginContext_H
