/*
 * Copyright (c) 2011, Dorian Scholz, TU Darmstadt
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

#include <rqt_rviz/rviz.h>

#include <pluginlib/class_list_macros.h>
#include <OGRE/OgreLogManager.h>

#include <QCloseEvent>
#include <QMenuBar>

namespace rqt_rviz {

RViz::RViz()
  : rqt_gui_cpp::Plugin()
  , context_(0)
  , widget_(0)
  , log_(0)
{
  setObjectName("RViz");
}

RViz::~RViz()
{
  Ogre::LogManager* log_manager = Ogre::LogManager::getSingletonPtr();
  if (log_manager && log_)
  {
    log_manager->destroyLog(log_);
  }
}

void RViz::initPlugin(qt_gui_cpp::PluginContext& context)
{
  context_ = &context;

  // prevent output of Ogre stuff to console
  Ogre::LogManager* log_manager = Ogre::LogManager::getSingletonPtr();
  if (!log_manager)
  {
    log_manager = new Ogre::LogManager();
  }
  QString filename = QString("rqt_rviz_ogre") + (context.serialNumber() > 1 ? QString::number(context.serialNumber()) : QString("")) + QString(".log");
  log_ = log_manager->createLog(filename.toStdString().c_str(), false, false);

  widget_ = new rviz::VisualizationFrame();

  // create own menu bar to disable native menu bars on Unity and Mac
  QMenuBar* menu_bar = new QMenuBar();
  menu_bar->setNativeMenuBar(false);
  widget_->setMenuBar(menu_bar);

  widget_->initialize();

  // disable quit action in menu bar
  QMenu* menu = 0;
  {
    // find first menu in menu bar
    const QObjectList& children = menu_bar->children();
    for (QObjectList::const_iterator it = children.begin(); !menu && it != children.end(); it++)
    {
      menu = dynamic_cast<QMenu*>(*it);
    }
  }
  if (menu)
  {
    // hide last action in menu
    const QObjectList& children = menu->children();
    if (!children.empty())
    {
      QAction* action = dynamic_cast<QAction*>(children.last());
      if (action)
      {
        action->setVisible(false);
      }
    }
  }

  widget_->setWindowTitle("RViz[*]");
  if (context.serialNumber() != 1)
  {
    widget_->setWindowTitle(widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
  }
  context.addWidget(widget_);

  // trigger deleteLater for plugin when widget or frame is closed
  widget_->installEventFilter(this);
}

bool RViz::eventFilter(QObject* watched, QEvent* event)
{
  if (watched == widget_ && event->type() == QEvent::Close)
  {
    event->ignore();
    context_->closePlugin();
    return true;
  }

  return QObject::eventFilter(watched, event);
}

}

PLUGINLIB_EXPORT_CLASS(rqt_rviz::RViz, rqt_gui_cpp::Plugin)
