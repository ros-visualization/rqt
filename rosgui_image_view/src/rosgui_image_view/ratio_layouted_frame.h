#ifndef rosgui_image_view__RatioLayoutedFrame_H
#define rosgui_image_view__RatioLayoutedFrame_H

#include <QFrame>
#include <QLayout>
#include <QLayoutItem>
#include <QPainter>
#include <QRect>
#include <QSize>

namespace rosgui_image_view {

/**
 * RatioLayoutedFrame is a layout containing a single frame with a fixed aspect ratio.
 * The default aspect ratio is 4:3.
 */
class RatioLayoutedFrame
  : public QFrame
{

  Q_OBJECT

public:

  RatioLayoutedFrame(QWidget* parent, Qt::WFlags flags = 0);

  virtual ~RatioLayoutedFrame();

  QRect getAspectRatioCorrectPaintArea();

  void resizeToFitAspectRatio();

  void setInnerFrameMinimumSize(const QSize& size);

  void setInnerFrameMaximumSize(const QSize& size);

  void setInnerFrameFixedSize(const QSize& size);

  void setAspectRatio(unsigned short width, unsigned short height);

private:

  static int greatestCommonDivisor(int a, int b);

  QSize aspect_ratio_;

};

}

#endif // rosgui_image_view__RatioLayoutedFrame_H
