#ifndef rosgui_image_view__ImageView_H
#define rosgui_image_view__ImageView_H

#include <rosgui_roscpp/plugin.h>

#include "ratio_layouted_frame.h"

#include <rosgui_image_view/ui_image_view.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>

#include <QDockWidget>
#include <QImage>
#include <QList>
#include <QString>

namespace rosgui_image_view {

class ImageView
  : public rosgui_roscpp::Plugin
{

  Q_OBJECT

public:

  ImageView();

  virtual void initPlugin(rosgui_cpp::PluginContext& context);

  virtual bool eventFilter(QObject* watched, QEvent* event);

  virtual void closePlugin();

  virtual void saveSettings(rosgui_cpp::Settings& global_settings, rosgui_cpp::Settings& perspective_settings);

  virtual void restoreSettings(rosgui_cpp::Settings& global_settings, rosgui_cpp::Settings& perspective_settings);

protected slots:

  virtual void updateTopicList();

protected:

  virtual QList<QString> getTopicList(const QSet<QString>& message_types, const QList<QString>& transports);

  virtual void selectTopic(const QString& topic);

protected slots:

  virtual void onTopicChanged(int index);

  virtual void onZoom1();

  virtual void onTopLevelChanged(bool topLevel);

protected:

  virtual void callbackImage(const sensor_msgs::Image::ConstPtr& msg);

  Ui::ImageViewDockWidget ui_;

  QDockWidget* widget_;

  image_transport::Subscriber subscriber_;

  QImage qimage_;

  RatioLayoutedFrame* raw_image_frame_;

};

}

#endif // rosgui_image_view__ImageView_H
