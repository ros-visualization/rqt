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

#ifndef RQT_GUI_CPP__NODELET_PLUGIN_PROVIDER_HPP_
#define RQT_GUI_CPP__NODELET_PLUGIN_PROVIDER_HPP_

#include <QSet>
#include <QThread>
#include <atomic>
#include <memory>
#include <string>

#include <qt_gui_cpp/plugin_context.hpp>
#include <qt_gui_cpp/ros_pluginlib_plugin_provider.hpp>

#include <rqt_gui_cpp/plugin.hpp>

#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/node.hpp>

namespace rqt_gui_cpp
{

class NodeletPluginProvider
  : public qt_gui_cpp::RosPluginlibPluginProvider<rqt_gui_cpp::Plugin>
{
public:
  NodeletPluginProvider(const QString & export_tag, const QString & base_class_type);

  ~NodeletPluginProvider() override;

  void unload(void * instance) override;

protected:
  void init_loader();

  std::shared_ptr<Plugin> create_plugin(
    const std::string & lookup_name,
    qt_gui_cpp::PluginContext * plugin_context) override;

  void init_plugin(
    const QString & plugin_id, qt_gui_cpp::PluginContext * plugin_context,
    qt_gui_cpp::Plugin * plugin) override;

  QSet<void *> instances_;

  bool loader_initialized_;

  // A shared node that is copied into each rqt_gui_cpp::PluginContext for use by rqt nodes
  std::shared_ptr<rclcpp::Node> node_;


  class RosSpinThread
    : public QThread
  {
public:
    explicit RosSpinThread(QObject * parent = nullptr);
    ~RosSpinThread() override;
    void run() override;
    std::atomic<bool> abort;
    // Create an executor that will be responsible for execution of callbacks for a set of nodes.
    // With this version, all callbacks will be called from within this thread (the main one).
    rclcpp::executors::MultiThreadedExecutor exec_;
  };

  RosSpinThread * ros_spin_thread_;
};
}  // namespace rqt_gui_cpp
#endif  // RQT_GUI_CPP__NODELET_PLUGIN_PROVIDER_HPP_
