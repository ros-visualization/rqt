/*
 * Copyright (c) 2026, Martin Pecka, Open Robotics
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
 *   * Neither the name of the copyright holder nor the names of its
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

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Open Robotics

#include <QApplication>

#include <pluginlib/class_list_macros.hpp>
#include <rqt_gui_cpp/plugin.hpp>

namespace test_rqt_gui_cpp
{

class Test
  : public rqt_gui_cpp::Plugin
{
public:
  void initPlugin(qt_gui_cpp::PluginContext & context) override
  {
    const auto app = QApplication::instance();
    if (app != nullptr) {
      std::string prefix;
      if (!context.argv().empty()) {
        for (int i = 0; i < context.argv().size(); i++) {
          const auto & arg = context.argv()[i];
          if (arg == "--prefix" && i + 1 < context.argv().size()) {
            prefix = context.argv()[i + 1].toStdString();
          }
        }
      }

      app->setProperty((prefix + "test_rqt_gui_cpp_argv").c_str(), context.argv());
      if (node_ != nullptr) {
        app->setProperty(
          (prefix + "test_rqt_gui_cpp_sim_time").c_str(),
          node_->get_parameter("use_sim_time").as_bool());
      }
    }
  }
};

class Te2st
  : public rqt_gui_cpp::Plugin
{
public:
  void initPlugin(qt_gui_cpp::PluginContext & context) override
  {
    const auto app = QApplication::instance();
    if (app != nullptr) {
      app->setProperty("test_rqt_gui_cpp_argv2", context.argv());
      if (node_ != nullptr) {
        app->setProperty(
          "test_rqt_gui_cpp_sim_time2",
          node_->get_parameter("use_sim_time").as_bool());
      }
    }
  }
};

}  // namespace test_rqt_gui_cpp

PLUGINLIB_EXPORT_CLASS(test_rqt_gui_cpp::Test, rqt_gui_cpp::Plugin)
PLUGINLIB_EXPORT_CLASS(test_rqt_gui_cpp::Te2st, rqt_gui_cpp::Plugin)
