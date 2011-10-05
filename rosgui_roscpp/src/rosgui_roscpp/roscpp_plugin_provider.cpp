#include "roscpp_plugin_provider.h"

#include "nodelet_plugin_provider.h"
#include <rosgui_roscpp/plugin.h>

#include <rosgui_cpp/plugin_provider.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <stdexcept>

namespace rosgui_roscpp {

RosCppPluginProvider::RosCppPluginProvider()
  : rosgui_cpp::CompositePluginProvider()
  , manager_name_("rosgui_roscpp_node")
  , callback_manager_(0)
{
  QList<PluginProvider*> plugin_providers;
  plugin_providers.append(new NodeletPluginProvider("rosgui", "rosgui_roscpp::Plugin", this));
  set_plugin_providers(plugin_providers);
}

QList<rosgui_cpp::PluginDescriptor*> RosCppPluginProvider::discover_descriptors()
{
  // initialize ROS nodelet manager
  int argc = 0;
  char** argv = 0;
  std::string name = "rosgui_roscpp_node";
  ros::init(argc, argv, name, ros::init_options::NoSigintHandler);

  if (!ros::master::check())
  {
    throw std::runtime_error("ROS master not found");
  }

  ros::start();

  callback_manager_ = new nodelet::detail::CallbackQueueManager();

  ros::NodeHandle nh(manager_name_);

  return rosgui_cpp::CompositePluginProvider::discover_descriptors();
}

}

PLUGINLIB_DECLARE_CLASS(rosgui_roscpp, RosCppPluginProvider, rosgui_roscpp::RosCppPluginProvider, rosgui_cpp::PluginProvider)
