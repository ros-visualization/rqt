#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys, math

from rosgui.QtBindingHelper import import_from_qt, Qwt
Qt, QTimer, SIGNAL, Slot = import_from_qt(['Qt', 'QTimer', 'SIGNAL', 'Slot'], 'QtCore')
QPen = import_from_qt('QPen', 'QtGui')

from numpy import arange, zeros, concatenate

# create real DataPlot class
class DataPlot(Qwt.QwtPlot):
    colors = [Qt.red,  Qt.blue,  Qt.green,  Qt.magenta]
    dataNumValuesSaved = 1000
    dataNumValuesPloted = 1000
    
    def __init__(self, *args, **kwargs):
        Qwt.QwtPlot.__init__(self, *args, **kwargs)
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
    
        markerAxisY = Qwt.QwtPlotMarker()
        markerAxisY.setLabelAlignment(Qt.AlignRight | Qt.AlignTop)
        markerAxisY.setLineStyle(Qwt.QwtPlotMarker.HLine)
        markerAxisY.setYValue(0.0)
        markerAxisY.attach(self)
    
        #self.setAxisTitle(Qwt.QwtPlot.xBottom, "Time")
        #self.setAxisTitle(Qwt.QwtPlot.yLeft, "Value")
    
        # Initialize data
        self.timeAxis = arange(self.dataNumValuesPloted)
        self.canvasDisplayHeight = 1000
        self.canvasDisplayWidth = self.canvas().width()
        self.dataOffsetX = self.dataNumValuesSaved - len(self.timeAxis)
        self.redraw()
        self.moveCanvas(0, 0)
    
        # init and start redraw timer
        self.timerRedraw = QTimer(self)
        self.timerRedraw.timeout.connect(self.redraw)
        if self.redrawTimerInterval:
            self.timerRedraw.start(self.redrawTimerInterval)

    
    def log(self, level, message):
        self.emit(SIGNAL('logMessage'), level, message)
    
    def setRedrawInterval(self, interval):
        self.redrawTimerInterval = interval
        if self.redrawTimerInterval:
            self.redrawOnEachUpdate = False
            self.redrawOnFullUpdate = False
            self.timerRedraw.start(self.redrawTimerInterval)
    
    def resize(self, *args):
        Qwt.QwtPlot.resize(self, *args)
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
        if self.curves.has_key(curveId):
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
        if (not self.pauseFlag) and self.curves.has_key(curveId):
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
        return self.curves.has_key(curveId)
    
    def redraw(self):
        for curveId in self.curves.keys():
            self.curves[curveId]['object'].setData(self.timeAxis, self.curves[curveId]['data'][self.dataOffsetX : self.dataOffsetX + len(self.timeAxis)])
            #self.curves[curveId]['object'].setStyle(Qwt.QwtPlotCurve.CurveStyle(3))
        self.replot()
    
    def rescale(self):
        canvasDisplayHeight = self.canvasDisplayHeight
        if canvasDisplayHeight > 2: # for bigger values, round up the value to get a nicer look for the y-axis
            canvasDisplayHeight = math.ceil(canvasDisplayHeight)
        self.setAxisScale(0, self.canvasOffsetY - (self.canvasDisplayHeight / 2), self.canvasOffsetY + (self.canvasDisplayHeight / 2), self.canvasDisplayHeight / 20)
        self.setAxisScale(2, 0, len(self.timeAxis))
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
        elif event.buttons() & Qt.RightButton: # right button zooms
            zoomFactor = max(-0.6, min(0.6, (self.lastCanvasY - canvasY) / 20.0 / 2.0))
            deltaY = (self.canvas().height() / 2.0) - self.pressedCanvasY
            self.moveCanvas(0, zoomFactor * deltaY * 1.0225)
            self.scaleAxisY(max(0.005, self.canvasDisplayHeight - (zoomFactor * self.canvasDisplayHeight)))
            self.rescaleAxisX(self.lastCanvasX - canvasX)
        self.lastCanvasX = canvasX
        self.lastCanvasY = canvasY
    
    def wheelEvent(self, event): # mouse wheel zooms the y-axis
        canvasY = event.y() - self.canvas().y()
        zoomFactor = max(-0.6, min(0.6, (event.delta() / 120) / 6.0))
        deltaY = (self.canvas().height() / 2.0) - canvasY
        self.moveCanvas(0, zoomFactor * deltaY * 1.0225)
        self.scaleAxisY(max(0.005, self.canvasDisplayHeight - zoomFactor * self.canvasDisplayHeight))


if __name__ == '__main__':
    from PyQt4.QtGui import QApplication

    app = QApplication(sys.argv)
    plot = DataPlot()
    plot.setRedrawInterval(30)
    plot.resize(700, 500)
    plot.show()
    plot.addCurve(0, '(x/5)^2')
    plot.addCurve(1, 'sin(x / 20) * 5000')
    for i in range(plot.dataNumValuesSaved):
        plot.updateValue(0, (i / 5.0) * (i / 5.0))
        plot.updateValue(1, math.sin(i / 20.0) * 5000)

    sys.exit(app.exec_())
