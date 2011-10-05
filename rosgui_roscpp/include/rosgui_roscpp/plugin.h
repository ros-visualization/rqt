#ifndef rosgui_roscpp__Plugin_H
#define rosgui_roscpp__Plugin_H

#include <rosgui_cpp/plugin.h>
#include <rosgui_cpp/plugin_context.h>
#include <rosgui_cpp/settings.h>

#include <nodelet/nodelet.h>

namespace rosgui_roscpp {

class Plugin
  : public rosgui_cpp::Plugin
  , public nodelet::Nodelet
{

public:

  Plugin()
    : rosgui_cpp::Plugin()
  {}

  virtual void initPlugin(rosgui_cpp::PluginContext& /*context*/)
  {}

  virtual void closePlugin()
  {}

  virtual void saveSettings(rosgui_cpp::Settings& /*global_settings*/, rosgui_cpp::Settings& /*perspective_settings*/)
  {}

  virtual void restoreSettings(rosgui_cpp::Settings& /*global_settings*/, rosgui_cpp::Settings& /*perspective_settings*/)
  {}

private:

  void onInit()
  {}

};

} // namespace

#endif // rosgui_roscpp__Plugin_H
