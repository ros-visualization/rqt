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

#ifndef rqt_image_view__RatioLayoutedFrame_H
#define rqt_image_view__RatioLayoutedFrame_H

#include <QFrame>
#include <QLayout>
#include <QLayoutItem>
#include <QPainter>
#include <QRect>
#include <QSize>

namespace rqt_image_view {

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

#endif // rqt_image_view__RatioLayoutedFrame_H
