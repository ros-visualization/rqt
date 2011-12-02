#ifndef rosgui_rviz__RViz_H
#define rosgui_rviz__RViz_H

#include <rosgui_roscpp/plugin.h>
#include <rviz/visualization_frame.h>

#include <QDockWidget>

namespace rosgui_rviz {

class RViz
  : public rosgui_roscpp::Plugin
{

  Q_OBJECT

public:

  RViz();

  virtual void initPlugin(rosgui_cpp::PluginContext& context);

  virtual bool eventFilter(QObject* watched, QEvent* event);

  virtual void closePlugin();

  virtual void saveSettings(rosgui_cpp::Settings& global_settings, rosgui_cpp::Settings& perspective_settings);

  virtual void restoreSettings(rosgui_cpp::Settings& global_settings, rosgui_cpp::Settings& perspective_settings);

protected:

  QDockWidget* widget_;

private:

  rviz::VisualizationFrame* frame_;

};

}

#endif // rosgui_rviz__RViz_H
