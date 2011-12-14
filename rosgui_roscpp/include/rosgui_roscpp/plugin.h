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
