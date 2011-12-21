namespace rosgui_cpp
{

class PluginContext
{
%TypeHeaderCode
#include <rosgui_cpp/plugin_context.h>
%End

public:

  PluginContext(QObject* obj, int serial_number);

  PluginContext(const rosgui_cpp::PluginContext& other);

  int serial_number();

  void add_widget(QWidget* widget, Qt::DockWidgetArea area);

  void update_widget_title(QWidget* widget);

  void remove_widget(QWidget* widget);

  void close_plugin();

};

};
