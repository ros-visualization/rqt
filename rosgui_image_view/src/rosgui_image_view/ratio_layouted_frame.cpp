#include "ratio_layouted_frame.h"

#include <assert.h>

namespace rosgui_image_view {

RatioLayoutedFrame::RatioLayoutedFrame(QWidget* parent, Qt::WFlags flags)
  : QFrame()
  , aspect_ratio_(4, 3)
{
}

RatioLayoutedFrame::~RatioLayoutedFrame()
{
}

void RatioLayoutedFrame::resizeToFitAspectRatio()
{
  QRect rect = contentsRect();

  // reduce longer edge to aspect ration
  double width = double(rect.width());
  double height = double(rect.height());
  if (width * aspect_ratio_.height() / height > aspect_ratio_.width())
  {
    // too large width
    width = height * aspect_ratio_.width() / aspect_ratio_.height();
    rect.setWidth(int(width));
  }
  else
  {
    // too large height
    height = width * aspect_ratio_.height() / aspect_ratio_.width();
    rect.setHeight(int(height));
  }

  // resize taking the border line into account
  int border = lineWidth();
  resize(rect.width() + 2 * border, rect.height() + 2 * border);
}

void RatioLayoutedFrame::setInnerFrameMinimumSize(const QSize& size)
{
  int border = lineWidth();
  QSize new_size = size;
  new_size += QSize(2 * border, 2 * border);
  setMinimumSize(new_size);
  update();
}

void RatioLayoutedFrame::setInnerFrameMaximumSize(const QSize& size)
{
  int border = lineWidth();
  QSize new_size = size;
  new_size += QSize(2 * border, 2 * border);
  setMaximumSize(new_size);
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

int RatioLayoutedFrame::greatestCommonDivisor(int a, int b)
{
  if (b==0)
  {
    return a;
  }
  return greatestCommonDivisor(b, a % b);
}

}
