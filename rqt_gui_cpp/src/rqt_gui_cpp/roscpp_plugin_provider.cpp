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

#include "roscpp_plugin_provider.hpp"

#include <sys/types.h>

#include <stdexcept>
#include <string.h>

#include "nodelet_plugin_provider.hpp"
#include <rqt_gui_cpp/plugin.hpp>

#include <qt_gui_cpp/plugin_provider.hpp>
#include <rclcpp/rclcpp.hpp>

#include <QCoreApplication>

#include <pluginlib/class_list_macros.hpp>

namespace rqt_gui_cpp
{

RosCppPluginProvider::RosCppPluginProvider()
: qt_gui_cpp::CompositePluginProvider()
  , rclcpp_initialized_(false)
{
  if (rclcpp::ok()) {
    rclcpp_initialized_ = true;
  }
  init_rclcpp(QCoreApplication::arguments());
  QList<PluginProvider *> plugin_providers;
  plugin_providers.append(new NodeletPluginProvider("rqt_gui", "rqt_gui_cpp::Plugin"));
  set_plugin_providers(plugin_providers);
}

RosCppPluginProvider::~RosCppPluginProvider()
{
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
}

void * RosCppPluginProvider::load(
  const QString & plugin_id,
  qt_gui_cpp::PluginContext * plugin_context)
{
  qDebug("RosCppPluginProvider::load(%s)", plugin_id.toStdString().c_str());
  init_rclcpp(plugin_context->argv());
  return qt_gui_cpp::CompositePluginProvider::load(plugin_id, plugin_context);
}

qt_gui_cpp::Plugin * RosCppPluginProvider::load_plugin(
  const QString & plugin_id,
  qt_gui_cpp::PluginContext * plugin_context)
{
  qDebug("RosCppPluginProvider::load_plugin(%s)", plugin_id.toStdString().c_str());
  init_rclcpp(plugin_context->argv());
  return qt_gui_cpp::CompositePluginProvider::load_plugin(plugin_id, plugin_context);
}

void RosCppPluginProvider::init_rclcpp(const QStringList& args)
{
  // convert plugin_context.argv() from a QStringList to a char**
  int argc = args.size();
  char** argv = new char*[argc];
  for (int i = 0; i < argc; i++)
  {
    argv[i] = strdup(args.at(i).toStdString().c_str());
  }

  init_rclcpp(argc, argv);

  for (int i = 0; i < argc; i++)
  {
    free(argv[i]);
  }
}
void RosCppPluginProvider::init_rclcpp(int argc, char** argv)
{
  // initialize ROS node once
  if (!rclcpp_initialized_) {
    // Initialize any global resources needed by the middleware and the client library.
    // You must call this before using any other part of the ROS system.
    // This should be called once per process.
    rclcpp::init(argc, argv);

    // Don't do this again in this process
    rclcpp_initialized_ = true;
  }
}
}  // namespace rqt_gui_cpp
PLUGINLIB_EXPORT_CLASS(rqt_gui_cpp::RosCppPluginProvider, qt_gui_cpp::PluginProvider)
