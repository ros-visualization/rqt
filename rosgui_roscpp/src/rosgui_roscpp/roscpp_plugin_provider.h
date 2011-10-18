#ifndef rosgui_roscpp__RosCppPluginProvider_H
#define rosgui_roscpp__RosCppPluginProvider_H

#include <rosgui_cpp/composite_plugin_provider.h>

#include <nodelet/detail/callback_queue_manager.h>

#include <string>

namespace rosgui_roscpp {

class RosCppPluginProvider
  : public rosgui_cpp::CompositePluginProvider
{

public:

  RosCppPluginProvider();

  virtual ~RosCppPluginProvider();

  virtual QList<rosgui_cpp::PluginDescriptor*> discover_descriptors();

  std::string manager_name_;

  nodelet::detail::CallbackQueueManager* callback_manager_;

};

}

#endif // rosgui_roscpp__RosCppPluginProvider_H
