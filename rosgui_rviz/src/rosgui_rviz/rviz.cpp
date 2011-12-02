#include "rviz.h"

#include <pluginlib/class_list_macros.h>
#include <OGRE/OgreLogManager.h>

#include <QCloseEvent>

namespace rosgui_rviz {

RViz::RViz()
  : rosgui_roscpp::Plugin()
  , widget_(0)
{
  setObjectName("RViz");
}

void RViz::initPlugin(rosgui_cpp::PluginContext& context)
{
  widget_ = new QDockWidget(context.main_window());

  widget_->setWindowTitle("RViz");
  if (context.serial_number() != 1)
  {
    widget_->setWindowTitle(widget_->windowTitle() + " (" + QString::number(context.serial_number()) + ")");
  }

  Ogre::LogManager* log_manager = new Ogre::LogManager();
  log_manager->createLog("Ogre.log", false, false, false);

  frame_ = new rviz::VisualizationFrame();
  // TODO: pass arguments
  frame_->initialize("", "", "", "", false);
  widget_->setWidget(frame_);

  context.main_window()->addDockWidget(Qt::RightDockWidgetArea, widget_);

  // trigger deleteLater for plugin when widget or frame is closed
  widget_->installEventFilter(this);
  frame_->installEventFilter(this);
}

bool RViz::eventFilter(QObject* watched, QEvent* event)
{
  if ((watched == widget_ || watched == frame_) && event->type() == QEvent::Close)
  {
    event->ignore();
    deletePluginLater();
    return true;
  }

  return QObject::eventFilter(watched, event);
}

void RViz::closePlugin()
{
  widget_->close();
  widget_->deleteLater();
}

void RViz::saveSettings(rosgui_cpp::Settings& global_settings, rosgui_cpp::Settings& perspective_settings)
{
}

void RViz::restoreSettings(rosgui_cpp::Settings& global_settings, rosgui_cpp::Settings& perspective_settings)
{
}

}

PLUGINLIB_DECLARE_CLASS(rosgui_rviz, RViz, rosgui_rviz::RViz, rosgui_roscpp::Plugin)
