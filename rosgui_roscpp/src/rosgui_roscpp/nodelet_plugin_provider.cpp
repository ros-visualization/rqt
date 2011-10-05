#include "nodelet_plugin_provider.h"

#include "roscpp_plugin_provider.h"

#include <nodelet/nodelet.h>

#include <stdexcept>

namespace rosgui_roscpp {

NodeletPluginProvider::NodeletPluginProvider(const QString& export_tag, const QString& base_class_type, RosCppPluginProvider* manager)
  : rosgui_cpp::RosPluginlibPluginProvider<rosgui_roscpp::Plugin>(export_tag, base_class_type)
  , manager_(manager)
{}

void NodeletPluginProvider::init_plugin(const QString& plugin_id, rosgui_cpp::PluginContext* plugin_context, rosgui_cpp::Plugin* plugin)
{
  qDebug("NodeletPluginProvider::init_plugin()");

  rosgui_roscpp::Plugin* nodelet = dynamic_cast<rosgui_roscpp::Plugin*>(plugin);
  if (!nodelet)
  {
    throw std::runtime_error("plugin is not a nodelet");
  }

  nodelet::M_string remappings;
  nodelet::V_string my_argv;
  std::string bond_id = plugin_id.toStdString() + "_" + plugin_context->serial_number().toStdString();
  boost::shared_ptr<bond::Bond> bond(new bond::Bond(manager_->manager_name_ + "/bond", bond_id));
  nodelet->init(bond_id, remappings, my_argv, manager_->callback_manager_, bond);

  rosgui_cpp::RosPluginlibPluginProvider<rosgui_roscpp::Plugin>::init_plugin(plugin_id, plugin_context, plugin);
}

}
