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
  : public QLayout
{

  Q_OBJECT

public:

  RatioLayoutedFrame(QWidget* parent, Qt::WFlags flags = 0);

  virtual ~RatioLayoutedFrame();

  void addItem(QLayoutItem* item);

  int count() const;

  QSize sizeHint() const;

  void setGeometry(const QRect& rect);

  QLayoutItem* itemAt(int index) const;

  QLayoutItem* takeAt(int index);

  QSize minimumSize() const;

  QSize maximumSize() const;

  QFrame* getFrame() const;

  QSize getInnerFrameSize() const;

  QSize getUnusedSpaceAroundFrame() const;

  void setInnerFrameMinimumSize(const QSize& size);

  void setInnerFrameMaximumSize(const QSize& size);

  void setInnerFrameFixedSize(const QSize& size);

  void setAspectRatio(unsigned short width, unsigned short height);

  void transformFramePainter(QPainter& painter, const QSize& native_size);

private:

  static int greatestCommonDivisor(int a, int b);

  QLayoutItem* frame_item_;

  QFrame* frame_;

  QSize aspect_ratio_;

};

}

#endif // rosgui_image_view__RatioLayoutedFrame_H
