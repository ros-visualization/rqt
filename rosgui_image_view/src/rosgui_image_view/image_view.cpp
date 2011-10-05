#include "image_view.h"

#include <pluginlib/class_list_macros.h>
#include <ros/master.h>
#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>

#include <QCloseEvent>
#include <QPainter>

namespace rosgui_image_view {

ImageView::ImageView()
  : rosgui_roscpp::Plugin()
  , widget_(0)
  , raw_image_frame_(0)
{
  setObjectName("ImageView");
}

void ImageView::initPlugin(rosgui_cpp::PluginContext& context)
{
  widget_ = new QDockWidget(context.main_window());
  ui_.setupUi(widget_);

  context.main_window()->addDockWidget(Qt::RightDockWidgetArea, widget_);

  // trigger deleteLater for plugin when widget is closed
  widget_->installEventFilter(this);

  raw_image_frame_ = new RatioLayoutedFrame(ui_.dockWidgetContents);
  raw_image_frame_->setMargin(0);
  raw_image_frame_->setSpacing(0);
  ui_.verticalLayout->addLayout(raw_image_frame_);
  raw_image_frame_->getFrame()->setFrameStyle(QFrame::Box | QFrame::Plain);
  raw_image_frame_->getFrame()->setLineWidth(1);
  raw_image_frame_->setInnerFrameMinimumSize(QSize(160, 120));
  raw_image_frame_->getFrame()->installEventFilter(this);

  updateTopicList();
  ui_.topics_combo_box->setCurrentIndex(ui_.topics_combo_box->findText(""));
  connect(ui_.topics_combo_box, SIGNAL(currentIndexChanged(int)), this, SLOT(onTopicChanged(int)));

  ui_.refresh_topics_push_button->setIcon(QIcon::fromTheme("view-refresh"));
  connect(ui_.refresh_topics_push_button, SIGNAL(pressed()), this, SLOT(updateTopicList()));

  ui_.zoom_1_push_button->setIcon(QIcon::fromTheme("zoom-original"));
  connect(ui_.zoom_1_push_button, SIGNAL(pressed()), this, SLOT(onZoom1()));
  onTopLevelChanged(widget_->isFloating());

  connect(widget_, SIGNAL(topLevelChanged(bool)), this, SLOT(onTopLevelChanged(bool)));
}

bool ImageView::eventFilter(QObject* watched, QEvent* event)
{
  if (watched == widget_ && event->type() == QEvent::Close)
  {
    event->ignore();
    widget_->removeEventFilter(this);
    deletePluginLater();
    return true;
  }
  else if (watched == raw_image_frame_->getFrame() && event->type() == QEvent::Paint)
  {
    if (!qimage_.isNull())
    {
      QPainter painter(raw_image_frame_->getFrame());
      // TODO: check if full draw is really necessary
      //QPaintEvent* paint_event = dynamic_cast<QPaintEvent*>(event);
      //painter.drawImage(paint_event->rect(), qimage_);
      painter.drawImage(raw_image_frame_->getFrame()->contentsRect(), qimage_);
    }
    return false;
  }

  return QObject::eventFilter(watched, event);
}

void ImageView::closePlugin()
{
  subscriber_.shutdown();
  widget_->close();
  widget_->deleteLater();
}

void ImageView::saveSettings(rosgui_cpp::Settings& global_settings, rosgui_cpp::Settings& perspective_settings)
{
  QString topic = ui_.topics_combo_box->currentText();
  //qDebug("ImageView::saveSettings() topic '%s'", topic.toStdString().c_str());
  perspective_settings.setValue("topic", topic);
}

void ImageView::restoreSettings(rosgui_cpp::Settings& global_settings, rosgui_cpp::Settings& perspective_settings)
{
  QString topic = perspective_settings.value("topic", "").toString();
  //qDebug("ImageView::restoreSettings() topic '%s'", topic.toStdString().c_str());
  selectTopic(topic);
}

void ImageView::updateTopicList()
{
  QSet<QString> message_types;
  message_types.insert("sensor_msgs/Image");

  // get declared transports
  QList<QString> transports;
  image_transport::ImageTransport it(getNodeHandle());
  std::vector<std::string> declared = it.getDeclaredTransports();
  for (std::vector<std::string>::const_iterator it = declared.begin(); it != declared.end(); it++)
  {
    //qDebug("ImageView::updateTopicList() declared transport '%s'", it->c_str());
    QString transport = it->c_str();

    // strip prefix from transport name
    QString prefix = "image_transport/";
    if (transport.startsWith(prefix))
    {
      transport = transport.mid(prefix.length());
    }
    transports.append(transport);
  }

  QString selected = ui_.topics_combo_box->currentText();

  // fill combo box
  QList<QString> topics = getTopicList(message_types, transports);
  topics.append("");
  qSort(topics);
  ui_.topics_combo_box->clear();
  for (QList<QString>::const_iterator it = topics.begin(); it != topics.end(); it++)
  {
    QString label(*it);
    label.replace(" ", "/");
    ui_.topics_combo_box->addItem(label, QVariant(*it));
  }

  // restore previous selection
  selectTopic(selected);
}

QList<QString> ImageView::getTopicList(const QSet<QString>& message_types, const QList<QString>& transports)
{
  ros::master::V_TopicInfo topic_info;
  ros::master::getTopics(topic_info);

  QSet<QString> all_topics;
  for (ros::master::V_TopicInfo::const_iterator it = topic_info.begin(); it != topic_info.end(); it++)
  {
    all_topics.insert(it->name.c_str());
  }

  QList<QString> topics;
  for (ros::master::V_TopicInfo::const_iterator it = topic_info.begin(); it != topic_info.end(); it++)
  {
    if (message_types.contains(it->datatype.c_str()))
    {
      QString topic = it->name.c_str();

      // add transport specific sub-topics
      bool has_transport = false;
      for (QList<QString>::const_iterator jt = transports.begin(); jt != transports.end(); jt++)
      {
        if (all_topics.contains(topic + "/" + *jt))
        {
          QString sub = topic + " " + *jt;
          topics.append(sub);
          //qDebug("ImageView::getTopicList() transport specific sub-topic '%s'", sub.toStdString().c_str());
          has_transport = true;
        }
      }

      // add raw topic only when no transport specific sub-topics exist
      if (!has_transport)
      {
        topics.append(topic);
        //qDebug("ImageView::getTopicList() raw topic '%s'", topic.toStdString().c_str());
      }
    }
  }
  return topics;
}

void ImageView::selectTopic(const QString& topic)
{
  int index = ui_.topics_combo_box->findText(topic);
  if (index == -1)
  {
    index = ui_.topics_combo_box->findText("");
  }
  ui_.topics_combo_box->setCurrentIndex(index);
}

void ImageView::onTopicChanged(int index)
{
  subscriber_.shutdown();

  QStringList parts = ui_.topics_combo_box->itemData(index).toString().split(" ");
  QString topic = parts.first();
  QString transport = parts.length() == 2 ? parts.last() : "raw";

  if (!topic.isEmpty())
  {
    image_transport::ImageTransport it(getNodeHandle());
    image_transport::TransportHints hints(transport.toStdString());
    subscriber_ = it.subscribe(topic.toStdString(), 1, &ImageView::callbackImage, this, hints);
    //qDebug("ImageView::onTopicChanged() to topic '%s' with transport '%s'", topic.toStdString().c_str(), subscriber_.getTransport().c_str());
  }
}

void ImageView::onZoom1()
{
  if (qimage_.isNull())
  {
    return;
  }

  QSize current = raw_image_frame_->getInnerFrameSize();
  QSize unused = raw_image_frame_->getUnusedSpaceAroundFrame();

  QSize offset = qimage_.size() - current - unused;
  if (!offset.isNull())
  {
    widget_->resize(widget_->size() + offset);
  }
}

void ImageView::onTopLevelChanged(bool topLevel)
{
  ui_.zoom_1_push_button->setDisabled(!topLevel);
}

void ImageView::callbackImage(const sensor_msgs::Image::ConstPtr& msg)
{
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
  }
  catch (cv_bridge::Exception& e)
  {
    qCritical("ImageView.callback_image() could not convert from '%s' to 'rgb8' (%s)", msg->encoding.c_str(), e.what());
    return;
  }

  QImage shared_image(cv_ptr->image.data, cv_ptr->image.cols, cv_ptr->image.rows, QImage::Format_RGB888);
  qimage_ = shared_image.copy();
  raw_image_frame_->setAspectRatio(cv_ptr->image.cols, cv_ptr->image.rows);
  raw_image_frame_->getFrame()->update();
}

}

PLUGINLIB_DECLARE_CLASS(rosgui_image_view, ImageView, rosgui_image_view::ImageView, rosgui_roscpp::Plugin)
