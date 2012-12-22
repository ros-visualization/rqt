#!/usr/bin/env python

# Copyright (c) 2011, Dorian Scholz
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

from python_qt_binding.QtCore import Slot, Qt
from python_qt_binding.QtGui import QColor, QVBoxLayout, QWidget

from pyqtgraph import PlotWidget, mkPen
import numpy


class PyQtGraphDataPlot(QWidget):
    _colors = [Qt.red, Qt.blue, Qt.magenta, Qt.cyan, Qt.green, Qt.darkYellow, Qt.black, Qt.darkRed, Qt.gray, Qt.darkCyan]

    def __init__(self, parent=None):
        super(PyQtGraphDataPlot, self).__init__(parent)
        self._plot_widget = PlotWidget()
        self._plot_widget.setBackground((255, 255, 255))
        self._plot_widget.setXRange(0, 10, padding=0)
        vbox = QVBoxLayout()
        vbox.addWidget(self._plot_widget)
        self.setLayout(vbox)

        self._color_index = 0
        self._curves = {}

    def add_curve(self, curve_id, curve_name, data_x, data_y):
        color = QColor(self._colors[self._color_index % len(self._colors)])
        self._color_index += 1
        pen = mkPen(color, width=1)
        plot = self._plot_widget.plot(name=curve_name, pen=pen)
        data_x = numpy.array(data_x)
        data_y = numpy.array(data_y)
        self._curves[curve_id] = {'x': data_x, 'y': data_y, 'plot': plot}
        self._update_legend()

    def remove_curve(self, curve_id):
        curve_id = str(curve_id)
        if curve_id in self._curves:
            self._plot_widget.removeItem(self._curves[curve_id]['plot'])
            del self._curves[curve_id]
            self._update_legend()

    def _update_legend(self):
        pass

    @Slot(str, list, list)
    def update_values(self, curve_id, x, y):
        curve = self._curves[curve_id]
        curve['x'] = numpy.append(curve['x'], x)
        curve['y'] = numpy.append(curve['y'], y)

    def redraw(self):
        # Set axis bounds
        x_range, _ = self._plot_widget.viewRange()
        x_delta = x_range[1] - x_range[0]
        x_max = 0
        for curve in self._curves.values():
            if len(curve['x']) == 0:
                continue

            x_max = max(x_max, curve['x'][-1])
            curve['plot'].setData(curve['x'], curve['y'])

        self._plot_widget.setXRange(x_max - x_delta, x_max, padding=0)
