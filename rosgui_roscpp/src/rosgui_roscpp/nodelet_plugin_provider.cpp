/*
 * Copyright (c) 2011, Dirk Thomas, TU Darmstadt
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the TU Darmstadt nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

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
  std::string bond_id = plugin_id.toStdString() + "_" + QString::number(plugin_context->serial_number()).toStdString();
  boost::shared_ptr<bond::Bond> bond(new bond::Bond(manager_->manager_name_ + "/bond", bond_id));
  nodelet->init(bond_id, remappings, my_argv, manager_->callback_manager_, bond);

  rosgui_cpp::RosPluginlibPluginProvider<rosgui_roscpp::Plugin>::init_plugin(plugin_id, plugin_context, plugin);
}

}
