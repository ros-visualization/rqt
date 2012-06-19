namespace qt_gui_cpp
{

class PluginContext
{
%TypeHeaderCode
#include <qt_gui_cpp/plugin_context.h>
%End

public:

  PluginContext(QObject* obj, int serial_number);

  PluginContext(const qt_gui_cpp::PluginContext& other);

  int serialNumber() const;

  void addWidget(QWidget* widget);

  void removeWidget(QWidget* widget);

  void closePlugin();

};

};
