#!/usr/bin/env python

# Copyright (c) 2011, Dorian Scholz, TU Darmstadt
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the TU Darmstadt nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# -*- coding: utf-8 -*-
from __future__ import division
import math
import sys

import qt_gui.qt_binding_helper  # @UnusedImport
from QtCore import QEvent, QPointF, QTimer, Qt, SIGNAL, Signal, Slot
from QtGui import QPen, QVector2D
import Qwt

from numpy import arange, zeros, concatenate


# create real DataPlot class
class DataPlot(Qwt.QwtPlot):
    mouseCoordinatesChanged = Signal(QPointF)
    colors = [Qt.red, Qt.blue, Qt.green, Qt.magenta]
    dataNumValuesSaved = 1000
    dataNumValuesPloted = 1000

    def __init__(self, *args):
        super(DataPlot, self).__init__(*args)
        self.setCanvasBackground(Qt.white)
        self.insertLegend(Qwt.QwtLegend(), Qwt.QwtPlot.BottomLegend)

        self.curves = {}
        self.pauseFlag = False
        self.dataOffsetX = 0
        self.canvasOffsetX = 0
        self.canvasOffsetY = 0
        self.lastCanvasX = 0
        self.lastCanvasY = 0
        self.pressedCanvasY = 0
        self.redrawOnEachUpdate = False
        self.redrawOnFullUpdate = True
        self.redrawTimerInterval = None
        self.redrawManually = False
        self.lastClickCoordinates = None

        markerAxisY = Qwt.QwtPlotMarker()
        markerAxisY.setLabelAlignment(Qt.AlignRight | Qt.AlignTop)
        markerAxisY.setLineStyle(Qwt.QwtPlotMarker.HLine)
        markerAxisY.setYValue(0.0)
        markerAxisY.attach(self)

        #self.setAxisTitle(Qwt.QwtPlot.xBottom, "Time")
        #self.setAxisTitle(Qwt.QwtPlot.yLeft, "Value")

        self.picker = Qwt.QwtPlotPicker(
            Qwt.QwtPlot.xBottom, Qwt.QwtPlot.yLeft, Qwt.QwtPicker.PolygonSelection,
            Qwt.QwtPlotPicker.PolygonRubberBand, Qwt.QwtPicker.AlwaysOn, self.canvas()
        )
        self.picker.setRubberBandPen(QPen(Qt.red))
        self.picker.setTrackerPen(QPen(Qt.red))

        # Initialize data
        self.timeAxis = arange(self.dataNumValuesPloted)
        self.canvasDisplayHeight = 1000
        self.canvasDisplayWidth = self.canvas().width()
        self.dataOffsetX = self.dataNumValuesSaved - len(self.timeAxis)
        self.redraw()
        self.moveCanvas(0, 0)
        self.canvas().setMouseTracking(True)
        self.canvas().installEventFilter(self)

        # init and start redraw timer
        self.timerRedraw = QTimer(self)
        self.timerRedraw.timeout.connect(self.redraw)
        if self.redrawTimerInterval:
            self.timerRedraw.start(self.redrawTimerInterval)

    def eventFilter(self, _, event):
        if event.type() == QEvent.MouseButtonRelease:
            x = self.invTransform(Qwt.QwtPlot.xBottom, event.pos().x())
            y = self.invTransform(Qwt.QwtPlot.yLeft, event.pos().y())
            self.lastClickCoordinates = QPointF(x, y)
        elif event.type() == QEvent.MouseMove:
            x = self.invTransform(Qwt.QwtPlot.xBottom, event.pos().x())
            y = self.invTransform(Qwt.QwtPlot.yLeft, event.pos().y())
            coords = QPointF(x, y)
            if self.picker.isActive() and self.lastClickCoordinates:
                toolTip = 'origin x: %.5f, y: %.5f' % (self.lastClickCoordinates.x(), self.lastClickCoordinates.y())
                delta = coords - self.lastClickCoordinates
                toolTip += '\ndelta x: %.5f, y: %.5f\nlength: %.5f' % (delta.x(), delta.y(), QVector2D(delta).length())
            else:
                toolTip = '(click to meassurement)'
            self.setToolTip(toolTip)
            self.mouseCoordinatesChanged.emit(coords)
        return False

    def log(self, level, message):
        self.emit(SIGNAL('logMessage'), level, message)

    def setRedrawInterval(self, interval):
        self.redrawTimerInterval = interval
        if self.redrawTimerInterval:
            self.redrawOnEachUpdate = False
            self.redrawOnFullUpdate = False
            self.timerRedraw.start(self.redrawTimerInterval)

    def resizeEvent(self, event):
        #super(DataPlot, self).resizeEvent(event)
        Qwt.QwtPlot.resizeEvent(self, event)
        self.rescale()

    def getCurves(self):
        return self.curves

    def addCurve(self, curveId, curveName):
        curveId = str(curveId)
        if self.curves.get(curveId):
            return
        curveObject = Qwt.QwtPlotCurve(curveName)
        curveObject.attach(self)
        curveObject.setPen(QPen(self.colors[len(self.curves.keys()) % len(self.colors)]))
        self.curves[curveId] = {
            'name': curveName,
            'data': zeros(self.dataNumValuesSaved),
            'object': curveObject,
        }

    def removeCurve(self, curveId):
        curveId = str(curveId)
        if curveId in self.curves:
            self.curves[curveId]['object'].hide()
            self.curves[curveId]['object'].attach(None)
            del self.curves[curveId]['object']
            del self.curves[curveId]

    def removeAllCurves(self):
        for curveId in self.curves.keys():
            self.removeCurve(curveId)
        self.clear()

    @Slot(str, float)
    def updateValue(self, curveId, value):
        curveId = str(curveId)
        # update data plot
        if (not self.pauseFlag) and curveId in self.curves:
            self.curves[curveId]['data'] = concatenate((self.curves[curveId]['data'][1:], self.curves[curveId]['data'][:1]), 1)
            self.curves[curveId]['data'][-1] = float(value)

            if not self.redrawManually:
                if self.redrawOnEachUpdate or (self.redrawOnFullUpdate and self.curves.keys()[0] == curveId):
                    self.redraw()

    @Slot(bool)
    def togglePause(self, enabled):
        self.pauseFlag = enabled

    def hasCurve(self, curveId):
        curveId = str(curveId)
        return curveId in self.curves

    def redraw(self):
        for curveId in self.curves.keys():
            self.curves[curveId]['object'].setData(self.timeAxis, self.curves[curveId]['data'][self.dataOffsetX: self.dataOffsetX + len(self.timeAxis)])
            #self.curves[curveId]['object'].setStyle(Qwt.QwtPlotCurve.CurveStyle(3))
        self.replot()

    def rescale(self):
        yNumTicks = self.height() / 40
        yLowerLimit = self.canvasOffsetY - (self.canvasDisplayHeight / 2)
        yUpperLimit = self.canvasOffsetY + (self.canvasDisplayHeight / 2)

        # calculate a fitting step size for nice, round tick labels, depending on the displayed value area
        yDelta = yUpperLimit - yLowerLimit
        exponent = int(math.log10(yDelta))
        presicion = -(exponent - 2)
        yStepSize = round(yDelta / yNumTicks, presicion)

        self.setAxisScale(Qwt.QwtPlot.yLeft, yLowerLimit, yUpperLimit, yStepSize)

        self.setAxisScale(Qwt.QwtPlot.xBottom, 0, len(self.timeAxis))
        self.redraw()

    def rescaleAxisX(self, deltaX):
        newLen = len(self.timeAxis) + deltaX
        newLen = max(10, min(newLen, self.dataNumValuesSaved))
        self.timeAxis = arange(newLen)
        self.dataOffsetX = max(0, min(self.dataOffsetX, self.dataNumValuesSaved - len(self.timeAxis)))
        self.rescale()

    def scaleAxisY(self, maxValue):
        self.canvasDisplayHeight = maxValue
        self.rescale()

    def moveCanvas(self, deltaX, deltaY):
        self.dataOffsetX += deltaX * len(self.timeAxis) / float(self.canvas().width())
        self.dataOffsetX = max(0, min(self.dataOffsetX, self.dataNumValuesSaved - len(self.timeAxis)))
        self.canvasOffsetX += deltaX * self.canvasDisplayWidth / self.canvas().width()
        self.canvasOffsetY += deltaY * self.canvasDisplayHeight / self.canvas().height()
        self.rescale()

    def mousePressEvent(self, event):
        self.lastCanvasX = event.x() - self.canvas().x()
        self.lastCanvasY = event.y() - self.canvas().y()
        self.pressedCanvasY = event.y() - self.canvas().y()

    def mouseMoveEvent(self, event):
        canvasX = event.x() - self.canvas().x()
        canvasY = event.y() - self.canvas().y()
        if event.buttons() & Qt.LeftButton: # left button moves the canvas
            deltaX = self.lastCanvasX - canvasX
            deltaY = canvasY - self.lastCanvasY
            self.moveCanvas(deltaX, deltaY)
        elif event.buttons() & Qt.RightButton:   # right button zooms
            zoomFactor = max(-0.6, min(0.6, (self.lastCanvasY - canvasY) / 20.0 / 2.0))
            deltaY = (self.canvas().height() / 2.0) - self.pressedCanvasY
            self.moveCanvas(0, zoomFactor * deltaY * 1.0225)
            self.scaleAxisY(max(0.005, self.canvasDisplayHeight - (zoomFactor * self.canvasDisplayHeight)))
            self.rescaleAxisX(self.lastCanvasX - canvasX)
        self.lastCanvasX = canvasX
        self.lastCanvasY = canvasY

    def wheelEvent(self, event):  # mouse wheel zooms the y-axis
        canvasY = event.y() - self.canvas().y()
        zoomFactor = max(-0.6, min(0.6, (event.delta() / 120) / 6.0))
        deltaY = (self.canvas().height() / 2.0) - canvasY
        self.moveCanvas(0, zoomFactor * deltaY * 1.0225)
        self.scaleAxisY(max(0.0005, self.canvasDisplayHeight - zoomFactor * self.canvasDisplayHeight))


if __name__ == '__main__':
    from PyQt4.QtGui import QApplication

    app = QApplication(sys.argv)
    plot = DataPlot()
    plot.setRedrawInterval(30)
    plot.resize(700, 500)
    plot.show()
    plot.addCurve(0, '(x/500)^2')
    plot.addCurve(1, 'sin(x / 20) * 500')
    for i in range(plot.dataNumValuesSaved):
        plot.updateValue(0, (i / 500.0) * (i / 5.0))
        plot.updateValue(1, math.sin(i / 20.0) * 500)

    sys.exit(app.exec_())
