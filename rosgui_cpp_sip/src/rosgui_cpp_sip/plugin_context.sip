namespace rosgui_cpp
{

class PluginContext
{
%TypeHeaderCode
#include <rosgui_cpp/plugin_context.h>
%End

public:

  PluginContext(QMainWindow* main_window, const QString& serial_number);

  QMainWindow* main_window();

  QString serial_number();

  const QMap<QString, QVariant>& attributes() const;

  QVariant attribute(const QString& key) const;

  void set_attribute(const QString& key, const QVariant& value);

};

};
