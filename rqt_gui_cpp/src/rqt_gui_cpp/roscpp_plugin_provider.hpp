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

#ifndef rqt_gui_cpp__RosCppPluginProvider_H
#define rqt_gui_cpp__RosCppPluginProvider_H

#include <qt_gui_cpp/composite_plugin_provider.h>

#include <QMessageBox>
#include <QThread>

#include <string>

namespace rqt_gui_cpp {

class RosCppPluginProvider
  : public qt_gui_cpp::CompositePluginProvider
{

public:

  RosCppPluginProvider();

  virtual ~RosCppPluginProvider();

  virtual void* load(const QString& plugin_id, qt_gui_cpp::PluginContext* plugin_context);

  virtual qt_gui_cpp::Plugin* load_plugin(const QString& plugin_id, qt_gui_cpp::PluginContext* plugin_context);

protected:

  void wait_for_master();

  void init_node();

  bool node_initialized_;

  QMessageBox* wait_for_master_dialog_;

  QThread* wait_for_master_thread_;

};

class WaitForMasterThread
  : public QThread
{
  Q_OBJECT
public:
  WaitForMasterThread(QObject* parent = 0);
  void run();
  bool abort;
signals:
  void master_found_signal(int r);
};

}

#endif // rqt_gui_cpp__RosCppPluginProvider_H
