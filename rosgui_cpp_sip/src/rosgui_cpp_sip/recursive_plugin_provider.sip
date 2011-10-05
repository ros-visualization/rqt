namespace rosgui_cpp
{

class RecursivePluginProvider
  : rosgui_cpp::CompositePluginProvider
{
%TypeHeaderCode
#include <rosgui_cpp/recursive_plugin_provider.h>
%End

public:

  RecursivePluginProvider(rosgui_cpp::RosPluginlibPluginProvider_ForPluginProviders* plugin_provider);

  virtual QMap<QString, QString> discover();

};

};
