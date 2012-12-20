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

#include <rqt_image_view/image_view.h>

#include <pluginlib/class_list_macros.h>
#include <ros/master.h>
#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <QMessageBox>
#include <QPainter>

namespace rqt_image_view {

ImageView::ImageView()
  : rqt_gui_cpp::Plugin()
  , widget_(0)
{
  setObjectName("ImageView");
}

void ImageView::initPlugin(qt_gui_cpp::PluginContext& context)
{
  widget_ = new QWidget();
  ui_.setupUi(widget_);

  if (context.serialNumber() > 1)
  {
    widget_->setWindowTitle(widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
  }
  context.addWidget(widget_);

  ui_.image_frame->installEventFilter(this);

  updateTopicList();
  ui_.topics_combo_box->setCurrentIndex(ui_.topics_combo_box->findText(""));
  connect(ui_.topics_combo_box, SIGNAL(currentIndexChanged(int)), this, SLOT(onTopicChanged(int)));

  ui_.refresh_topics_push_button->setIcon(QIcon::fromTheme("view-refresh"));
  connect(ui_.refresh_topics_push_button, SIGNAL(pressed()), this, SLOT(updateTopicList()));

  ui_.zoom_1_push_button->setIcon(QIcon::fromTheme("zoom-original"));
  connect(ui_.zoom_1_push_button, SIGNAL(toggled(bool)), this, SLOT(onZoom1(bool)));
  
  connect(ui_.dynamic_range_check_box, SIGNAL(toggled(bool)), this, SLOT(onDynamicRange(bool)));
}

bool ImageView::eventFilter(QObject* watched, QEvent* event)
{
  if (watched == ui_.image_frame && event->type() == QEvent::Paint)
  {
    QPainter painter(ui_.image_frame);
    if (!qimage_.isNull())
    {
      ui_.image_frame->resizeToFitAspectRatio();
      // TODO: check if full draw is really necessary
      //QPaintEvent* paint_event = dynamic_cast<QPaintEvent*>(event);
      //painter.drawImage(paint_event->rect(), qimage_);
      qimage_mutex_.lock();
      painter.drawImage(ui_.image_frame->contentsRect(), qimage_);
      qimage_mutex_.unlock();
    } else {
      // default image with gradient
      QLinearGradient gradient(0, 0, ui_.image_frame->frameRect().width(), ui_.image_frame->frameRect().height());
      gradient.setColorAt(0, Qt::white);
      gradient.setColorAt(1, Qt::black);
      painter.setBrush(gradient);
      painter.drawRect(0, 0, ui_.image_frame->frameRect().width() + 1, ui_.image_frame->frameRect().height() + 1);
    }
    return false;
  }

  return QObject::eventFilter(watched, event);
}

void ImageView::shutdownPlugin()
{
  subscriber_.shutdown();
}

void ImageView::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
  QString topic = ui_.topics_combo_box->currentText();
  //qDebug("ImageView::saveSettings() topic '%s'", topic.toStdString().c_str());
  instance_settings.setValue("topic", topic);
  instance_settings.setValue("zoom1", ui_.zoom_1_push_button->isChecked());
  instance_settings.setValue("dynamic_range", ui_.dynamic_range_check_box->isChecked());
  instance_settings.setValue("max_range", ui_.max_range_double_spin_box->value());
}

void ImageView::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
  bool zoom1_checked = instance_settings.value("zoom1", false).toBool();
  ui_.zoom_1_push_button->setChecked(zoom1_checked);

  bool dynamic_range_checked = instance_settings.value("dynamic_range", false).toBool();
  ui_.dynamic_range_check_box->setChecked(dynamic_range_checked);

  double max_range = instance_settings.value("max_range", ui_.max_range_double_spin_box->value()).toDouble();
  ui_.max_range_double_spin_box->setValue(max_range);

  QString topic = instance_settings.value("topic", "").toString();
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

      // add raw topic
      topics.append(topic);
      //qDebug("ImageView::getTopicList() raw topic '%s'", topic.toStdString().c_str());
      
      // add transport specific sub-topics
      for (QList<QString>::const_iterator jt = transports.begin(); jt != transports.end(); jt++)
      {
        if (all_topics.contains(topic + "/" + *jt))
        {
          QString sub = topic + " " + *jt;
          topics.append(sub);
          //qDebug("ImageView::getTopicList() transport specific sub-topic '%s'", sub.toStdString().c_str());
        }
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

  // reset image on topic change
  qimage_ = QImage();
  ui_.image_frame->update();

  QStringList parts = ui_.topics_combo_box->itemData(index).toString().split(" ");
  QString topic = parts.first();
  QString transport = parts.length() == 2 ? parts.last() : "raw";

  if (!topic.isEmpty())
  {
    image_transport::ImageTransport it(getNodeHandle());
    image_transport::TransportHints hints(transport.toStdString());
    try {
      subscriber_ = it.subscribe(topic.toStdString(), 1, &ImageView::callbackImage, this, hints);
      //qDebug("ImageView::onTopicChanged() to topic '%s' with transport '%s'", topic.toStdString().c_str(), subscriber_.getTransport().c_str());
    } catch (image_transport::TransportLoadException& e) {
      QMessageBox::warning(widget_, tr("Loading image transport plugin failed"), e.what());
    }
  }
}

void ImageView::onZoom1(bool checked)
{
  if (checked)
  {
    if (qimage_.isNull())
    {
      return;
    }
    ui_.image_frame->setInnerFrameFixedSize(qimage_.size());
    widget_->resize(ui_.image_frame->size());
    widget_->setMinimumSize(widget_->sizeHint());
    widget_->setMaximumSize(widget_->sizeHint());
  } else {
    ui_.image_frame->setInnerFrameMinimumSize(QSize(80, 60));
    ui_.image_frame->setMaximumSize(QSize(QWIDGETSIZE_MAX, QWIDGETSIZE_MAX));
    widget_->setMinimumSize(QSize(80, 60));
    widget_->setMaximumSize(QSize(QWIDGETSIZE_MAX, QWIDGETSIZE_MAX));
  }
}

void ImageView::onDynamicRange(bool checked)
{
  ui_.max_range_double_spin_box->setEnabled(!checked);
}

void ImageView::callbackImage(const sensor_msgs::Image::ConstPtr& msg)
{
  try
  {
    // First check if we have a color format. If so, use the current tools for color conversion
    if (sensor_msgs::image_encodings::isColor(msg->encoding) || sensor_msgs::image_encodings::isMono(msg->encoding))
    {
      cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
      conversion_mat_ = cv_ptr->image;
    }
    else
    {
      cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg);
      if (msg->encoding == "CV_8UC3")
      {
        // assuming it is rgb
        conversion_mat_ = cv_ptr->image;
      } else if (msg->encoding == "8UC1") {
        // convert gray to rgb
        cv::cvtColor(cv_ptr->image, conversion_mat_, CV_GRAY2RGB);
      } else if (msg->encoding == "16UC1" || msg->encoding == "32FC1") {
        // scale / quantify
        double min = 0;
        double max = ui_.max_range_double_spin_box->value();
        if (msg->encoding == "16UC1") max *= 1000;
        if (ui_.dynamic_range_check_box->isChecked())
        {
          // dynamically adjust range based on min/max in image
          cv::minMaxLoc(cv_ptr->image, &min, &max);
          if (min == max) {
            // completely homogeneous images are displayed in gray
            min = 0;
            max = 2;
          }
        }
        cv::Mat img_scaled_8u;
        cv::Mat(cv_ptr->image-min).convertTo(img_scaled_8u, CV_8UC1, 255. / (max - min));
        cv::cvtColor(img_scaled_8u, conversion_mat_, CV_GRAY2RGB);
      } else {
        qWarning("ImageView.callback_image() unhandled image encoding '%s'", msg->encoding.c_str());
        qimage_ = QImage();
        return;
      }
    }
  }
  catch (cv_bridge::Exception& e)
  {
    qWarning("ImageView.callback_image() could not convert image from '%s' to 'rgb8' (%s)", msg->encoding.c_str(), e.what());
    qimage_ = QImage();
    return;
  }

  // copy temporary image as it uses the conversion_mat_ for storage which is asynchronously overwritten in the next callback invocation
  QImage image(conversion_mat_.data, conversion_mat_.cols, conversion_mat_.rows, QImage::Format_RGB888);
  qimage_mutex_.lock();
  qimage_ = image.copy();
  qimage_mutex_.unlock();

  ui_.image_frame->setAspectRatio(qimage_.width(), qimage_.height());
  if (!ui_.zoom_1_push_button->isEnabled())
  {
    ui_.zoom_1_push_button->setEnabled(true);
    onZoom1(ui_.zoom_1_push_button->isChecked());
  }
  ui_.image_frame->update();
}

}

PLUGINLIB_EXPORT_CLASS(rqt_image_view::ImageView, rqt_gui_cpp::Plugin)
