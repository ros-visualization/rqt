#!/usr/bin/env python

# Copyright (c) 2011, Ye Cheng, Dorian Scholz
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

import collections
import qt_gui.qt_binding_helper  # @UnusedImport
from QtCore import Slot, Qt
from QtGui import QWidget, QVBoxLayout, QSizePolicy, QColor

import matplotlib
if matplotlib.__version__ < '1.1.0':
    raise RuntimeError('A newer matplotlib is required (at least 1.1.0)')

try:
    from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
except ImportError:
    # work around bug in dateutil
    import sys
    import thread
    sys.modules['_thread'] = thread
    from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt4agg import NavigationToolbar2QTAgg as NavigationToolbar
from matplotlib.figure import Figure

import numpy


class MatDataPlot(QWidget):
    class Canvas(FigureCanvas):
        """Ultimately, this is a QWidget (as well as a FigureCanvasAgg, etc.)."""
        def __init__(self, parent=None):
            fig = Figure()
            self.axes = fig.add_subplot(111)
            self.axes.grid(True, color='gray')
            super(MatDataPlot.Canvas, self).__init__(fig)
            self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            self.updateGeometry()

    _colors = [Qt.red, Qt.blue, Qt.magenta, Qt.cyan, Qt.green, Qt.darkYellow, Qt.black, Qt.darkRed, Qt.gray, Qt.darkCyan]

    def __init__(self, parent=None):
        super(MatDataPlot, self).__init__(parent)
        self._canvas = MatDataPlot.Canvas()
        self._toolbar = NavigationToolbar(self._canvas, self._canvas)
        vbox = QVBoxLayout()
        vbox.addWidget(self._toolbar)
        vbox.addWidget(self._canvas)
        self.setLayout(vbox)

        self._color_index = 0
        self._curves = {}

    def add_curve(self, curve_id, curve_name, data_x, data_y):
        data_x = collections.deque(data_x)
        data_y = collections.deque(data_y)
        color = QColor(self._colors[self._color_index % len(self._colors)])
        self._color_index += 1
        plot = self._canvas.axes.plot(data_x, data_y, label=curve_name, linewidth=1, picker=5, color=color.name())[0]
        self._curves[curve_id] = (data_x, data_y, plot)
        self._canvas.axes.legend(self._curves.keys())

    def remove_curve(self, curve_id):
        curve_id = str(curve_id)
        if curve_id in self._curves:
            self._curves[curve_id][2].remove()
            del self._curves[curve_id]
            self._canvas.axes.legend(self._curves.keys())

    @Slot(str, list, list)
    def update_values(self, curve_id, x, y):
        data_x, data_y, _ = self._curves[curve_id]
        data_x.extend(x)
        data_y.extend(y)

    def redraw(self):
        self._canvas.axes.grid(True, color='gray')
        # Set axis bounds
        ymin = ymax = None
        xmax = xmin = 0
        for curve in self._curves.values():
            data_x, data_y, plot = curve
            if len(data_x) == 0:
                continue

            xmax = data_x[-1]
            xmin = xmax - 5

            if ymin is None:
                ymin = min(data_y)
                ymax = max(data_y)
            else:
                ymin = min(min(data_y), ymin)
                ymax = max(max(data_y), ymax)

            # pad the min/max
            delta = max(ymax - ymin, 0.1)
            ymin -= .05 * delta
            ymax += .05 * delta

            self._canvas.axes.set_xbound(lower=xmin, upper=xmax)
            self._canvas.axes.set_ybound(lower=ymin, upper=ymax)

        # Set plot data on current axes
        for curve in self._curves.values():
            data_x, data_y, plot = curve
            plot.set_data(numpy.array(data_x), numpy.array(data_y))

        self._canvas.draw()
