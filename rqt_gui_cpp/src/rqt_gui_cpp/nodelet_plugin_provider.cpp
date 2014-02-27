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

#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <stdexcept>
#include <boost/bind.hpp>

namespace rqt_gui_cpp {

NodeletPluginProvider::NodeletPluginProvider(const QString& export_tag, const QString& base_class_type)
  : qt_gui_cpp::RosPluginlibPluginProvider<rqt_gui_cpp::Plugin>(export_tag, base_class_type)
  , loader_(0)
  , ros_spin_thread_(0)
{}

NodeletPluginProvider::~NodeletPluginProvider()
{
  if (loader_)
  {
    delete loader_;
  }
}

void NodeletPluginProvider::unload(void* instance)
{
  qDebug("NodeletPluginProvider::unload()");
  if (!instances_.contains(instance))
  {
    qCritical("NodeletPluginProvider::unload() instance not found");
    return;
  }

  QString nodelet_name = instances_[instance];
  bool unloaded = loader_->unload(nodelet_name.toStdString());
  if (!unloaded)
  {
    qCritical("NodeletPluginProvider::unload() '%s' failed", nodelet_name.toStdString().c_str());
  }

  // stop and garbage ros spin thread if not needed anymore
  if (loader_->listLoadedNodelets().empty())
  {
    shutdown();
  }

  qt_gui_cpp::RosPluginlibPluginProvider<rqt_gui_cpp::Plugin>::unload(instance);
}

void NodeletPluginProvider::shutdown()
{
  if (ros_spin_thread_ != 0)
  {
    ros_spin_thread_->abort = true;
    ros_spin_thread_->wait();
    ros_spin_thread_->deleteLater();
    ros_spin_thread_ = 0;
  }
}

void NodeletPluginProvider::init_loader()
{
  // initialize nodelet Loader once
  if (loader_ == 0)
  {
    loader_ = new nodelet::Loader(boost::bind(&NodeletPluginProvider::create_instance, this, _1));
  }

  // spawn ros spin thread
  if (ros_spin_thread_ == 0)
  {
    ros_spin_thread_ = new RosSpinThread(this);
    ros_spin_thread_->start();
  }
}

boost::shared_ptr<Plugin> NodeletPluginProvider::create_plugin(const std::string& lookup_name, qt_gui_cpp::PluginContext* plugin_context)
{
  init_loader();

  nodelet::M_string remappings;
  nodelet::V_string my_argv;
  std::string nodelet_name = lookup_name + "_" + QString::number(plugin_context->serialNumber()).toStdString();
  instance_.reset();
  qDebug("NodeletPluginProvider::create_plugin() load %s", lookup_name.c_str());
  bool loaded = loader_->load(nodelet_name, lookup_name, remappings, my_argv);
  if (loaded)
  {
    qDebug("NodeletPluginProvider::create_plugin() loaded");
    instances_[&*instance_] = nodelet_name.c_str();
  }
  boost::shared_ptr<rqt_gui_cpp::Plugin> instance = instance_;
  instance_.reset();
  return instance;
}

boost::shared_ptr<nodelet::Nodelet> NodeletPluginProvider::create_instance(const std::string& lookup_name)
{
  instance_ = qt_gui_cpp::RosPluginlibPluginProvider<rqt_gui_cpp::Plugin>::create_plugin(lookup_name);
  return instance_;
}

void NodeletPluginProvider::init_plugin(const QString& plugin_id, qt_gui_cpp::PluginContext* plugin_context, qt_gui_cpp::Plugin* plugin)
{
  qDebug("NodeletPluginProvider::init_plugin()");

  rqt_gui_cpp::Plugin* nodelet = dynamic_cast<rqt_gui_cpp::Plugin*>(plugin);
  if (!nodelet)
  {
    throw std::runtime_error("plugin is not a nodelet");
  }

  qt_gui_cpp::RosPluginlibPluginProvider<rqt_gui_cpp::Plugin>::init_plugin(plugin_id, plugin_context, plugin);
}

NodeletPluginProvider::RosSpinThread::RosSpinThread(QObject* parent)
  : QThread(parent)
  , abort(false)
{}

NodeletPluginProvider::RosSpinThread::~RosSpinThread()
{}

void NodeletPluginProvider::RosSpinThread::run()
{
  while (!abort)
  {
    ros::getGlobalCallbackQueue()->callOne(ros::WallDuration(0.1));
  }
}

}
