namespace rosgui_cpp
{

class MainWindowInterface
{
%TypeHeaderCode
#include <rosgui_cpp/main_window_interface.h>
%End

public:

  MainWindowInterface(QWidget* obj);

  virtual void set_plugin_instance(rosgui_cpp::PluginBridge* plugin_instance);

};

};
