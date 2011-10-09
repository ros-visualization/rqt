namespace rosgui_cpp
{

class PluginContext
{
%TypeHeaderCode
#include <rosgui_cpp/plugin_context.h>
%End

public:

  PluginContext(rosgui_cpp::MainWindowInterface* main_window, int serial_number);

  rosgui_cpp::MainWindowInterface* main_window();

  int serial_number();

  const QMap<QString, QVariant>& attributes() const;

  QVariant attribute(const QString& key) const;

  void set_attribute(const QString& key, const QVariant& value);

};

};
