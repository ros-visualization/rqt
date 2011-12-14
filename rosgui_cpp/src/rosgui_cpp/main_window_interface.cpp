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

#include <rosgui_cpp/main_window_interface.h>

#include <stdexcept>

namespace rosgui_cpp {

MainWindowInterface::MainWindowInterface(QWidget* obj)
  : QWidget(obj)
  , proxy_(obj)
{}

MainWindowInterface::MainWindowInterface(const MainWindowInterface& other)
  : QWidget(other.parentWidget())
  , proxy_(other.parentWidget())
{}

void MainWindowInterface::addDockWidget(Qt::DockWidgetArea area, QDockWidget* dock_widget)
{
  bool rc = proxy_.invokeMethod("addDockWidget", Q_ARG(int, area), Q_ARG(QDockWidget*, dock_widget));
  if (!rc) throw std::runtime_error("MainWindowInterface::addDockWidget() invoke method failed");
}

void MainWindowInterface::removeDockWidget(QDockWidget* dock_widget)
{
  bool rc = proxy_.invokeMethod("removeDockWidget", Q_ARG(QDockWidget*, dock_widget));
  if (!rc) throw std::runtime_error("MainWindowInterface::removeDockWidget() invoke method failed");
}

void MainWindowInterface::set_plugin_instance(PluginBridge* plugin_instance)
{
  bool rc = proxy_.invokeMethod("set_plugin_instance", Q_ARG(QObject*, plugin_instance));
  if (!rc) throw std::runtime_error("MainWindowInterface::set_plugin_instance() invoke method failed");
}

} // namespace
