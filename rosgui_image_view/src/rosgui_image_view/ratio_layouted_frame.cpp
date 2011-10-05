#include "ratio_layouted_frame.h"

#include <assert.h>

namespace rosgui_image_view {

RatioLayoutedFrame::RatioLayoutedFrame(QWidget* parent, Qt::WFlags flags)
  : QLayout()
  , frame_item_(0)
  , frame_(0)
  , aspect_ratio_(4, 3)
{
  frame_ = new QFrame(parent, flags);
  addItem(new QWidgetItem(frame_));
}

RatioLayoutedFrame::~RatioLayoutedFrame()
{
  if (frame_ != 0)
  {
    delete frame_;
    frame_ = 0;
  }
  if (frame_item_ != 0)
  {
    delete frame_item_;
    frame_item_ = 0;
  }
}

void RatioLayoutedFrame::addItem(QLayoutItem* item)
{
  assert(frame_item_ == 0);
  frame_item_ = item;
}

int RatioLayoutedFrame::count() const
{
  return frame_item_ != 0 ? 1 : 0;
}

QSize RatioLayoutedFrame::sizeHint() const
{
  QSize size(1, 1);
  size = size.expandedTo(frame_->minimumSize());
  size = size.expandedTo(frame_->sizeHint());
  int border = frame_->lineWidth();
  size += QSize(2 * border, 2 * border);
  return size;
}

void RatioLayoutedFrame::setGeometry(const QRect& rect)
{
  QLayout::setGeometry(rect);

  // remove border for computation
  int border = frame_->lineWidth();
  int w = rect.width() - 2 * border;
  int h = rect.height() - 2 * border;

  // steps in size of aspect ration
  w -= w % aspect_ratio_.width();
  h -= h % aspect_ratio_.height();

  // reduce longer edge to aspect ration
  double width = double(w);
  double height = double(h);
  if (width * aspect_ratio_.height() / height > aspect_ratio_.width())
  {
    // too large width
    width = height * aspect_ratio_.width() / aspect_ratio_.height();
  }
  else
  {
    // too large height
    height = width * aspect_ratio_.height() / aspect_ratio_.width();
  }
  // readd border after computation
  w = int(width) + 2 * border;
  h = int(height) + 2 * border;

  // set smaller geometry respecting the aspect ratio
  QRect geom(rect.x(), rect.y(), w, h);
  frame_->setGeometry(geom);
}

QLayoutItem* RatioLayoutedFrame::itemAt(int index) const
{
  if (index == 0)
  {
    return frame_item_;
  }
  return 0;
}

QLayoutItem* RatioLayoutedFrame::takeAt(int index)
{
  assert(false);
  return 0;
}

QSize RatioLayoutedFrame::minimumSize() const
{
  return frame_->minimumSize();
}

QSize RatioLayoutedFrame::maximumSize() const
{
  return frame_->maximumSize();
}

QFrame* RatioLayoutedFrame::getFrame() const
{
  return frame_;
}

QSize RatioLayoutedFrame::getInnerFrameSize() const
{
  QSize size = frame_->size();
  int border = frame_->lineWidth();
  size -= QSize(2 * border, 2 * border);
  return size;
}

QSize RatioLayoutedFrame::getUnusedSpaceAroundFrame() const
{
  return geometry().size() - frame_->size();
}

void RatioLayoutedFrame::setInnerFrameMinimumSize(const QSize& size)
{
  int border = frame_->lineWidth();
  QSize new_size = size;
  new_size += QSize(2 * border, 2 * border);
  frame_->setMinimumSize(new_size);
  update();
}

void RatioLayoutedFrame::setInnerFrameMaximumSize(const QSize& size)
{
  int border = frame_->lineWidth();
  QSize new_size = size;
  new_size += QSize(2 * border, 2 * border);
  frame_->setMaximumSize(new_size);
  update();
}

void RatioLayoutedFrame::setInnerFrameFixedSize(const QSize& size)
{
  setInnerFrameMinimumSize(size);
  setInnerFrameMaximumSize(size);
}

void RatioLayoutedFrame::setAspectRatio(unsigned short width, unsigned short height)
{
  int divisor = greatestCommonDivisor(width, height);
  aspect_ratio_.setWidth(width / divisor);
  aspect_ratio_.setHeight(height / divisor);
}

void RatioLayoutedFrame::transformFramePainter(QPainter& painter, const QSize& native_size)
{
  int border = frame_->lineWidth();
  painter.translate(QPoint(border, border));
  QSize frame_size = getInnerFrameSize();
  painter.scale(1.0 * frame_size.width() / native_size.width(), 1.0 * frame_size.height() / native_size.height());
}

int RatioLayoutedFrame::greatestCommonDivisor(int a, int b)
{
  if (b==0)
  {
    return a;
  }
  return greatestCommonDivisor(b, a % b);
}

}
