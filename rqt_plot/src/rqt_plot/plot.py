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

import os
import roslib
roslib.load_manifest('rqt_plot')

from python_qt_binding.QtCore import qDebug
from rqt_gui_py.plugin import Plugin
from qt_gui_py_common.simple_settings_dialog import SimpleSettingsDialog
from plot_widget import PlotWidget

try:
    qDebug('rqt_plot.plot: importing PyQtGraphDataPlot')
    from pyqtgraph_data_plot import PyQtGraphDataPlot
except ImportError:
    qDebug('rqt_plot.plot: import of PyQtGraphDataPlot failed')
    PyQtGraphDataPlot = None

try:
    qDebug('rqt_plot.plot: importing MatDataPlot')
    from mat_data_plot import MatDataPlot
except ImportError:
    qDebug('rqt_plot.plot: import of MatDataPlot failed')
    MatDataPlot = None

try:
    qDebug('rqt_plot.plot: importing QwtDataPlot')
    from qwt_data_plot import QwtDataPlot
except ImportError:
    qDebug('rqt_plot.plot: import of QwtDataPlot failed')
    QwtDataPlot = None

class Plot(Plugin):
    # plot types in order of priority
    plot_types = [
        {
            'title': 'PyQtGraph',
            'widget_class': PyQtGraphDataPlot,
            'description': 'Based on PyQtGraph\n- installer: http://luke.campagnola.me/code/pyqtgraph',
            'enabled': PyQtGraphDataPlot is not None,
        }, 
        {
            'title': 'MatPlot',
            'widget_class': MatDataPlot,
            'description': 'Based on MatPlotLib\n- needs most CPU\n- needs matplotlib',
            'enabled': MatDataPlot is not None,
        }, 
        {
            'title': 'QwtPlot',
            'widget_class': QwtDataPlot,
            'description': 'Based on QwtPlot\n- does not use timestamps\n- uses least CPU\n- needs Python Qwt bindings',
            'enabled': QwtDataPlot is not None,
        }, 
    ]
    def __init__(self, context):
        super(Plot, self).__init__(context)
        self.setObjectName('Plot')

        self._plot_type_index = 0
        self._context = context
        self._widget = PlotWidget()
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

    def _switch_data_plot_widget(self, plot_type_index):
        # check if selected plot type is available
        if not self.plot_types[plot_type_index]['enabled']:
            # if not, check for any other available plot type
            for index, plot_type in enumerate(self.plot_types):
                if plot_type['enabled']:
                    plot_type_index = index
                    break
            else:
                print 'No usable plot type found. Install at least one of: PyQtGraph, MatPlotLib or Python-Qwt5'
                return
            
        self._plot_type_index = plot_type_index
        selected_plot = self.plot_types[plot_type_index]
        
        self._widget.switch_data_plot_widget(selected_plot['widget_class'](self._widget))
        self._widget.setWindowTitle(selected_plot['title'])
        if self._context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % self._context.serial_number()))
        
    def save_settings(self, plugin_settings, instance_settings):
        instance_settings.set_value('plot_type', self._plot_type_index)

    def restore_settings(self, plugin_settings, instance_settings):
        self._switch_data_plot_widget(int(instance_settings.value('plot_type', 0)))

    def trigger_configuration(self):
        dialog = SimpleSettingsDialog(title='Plot Options')
        dialog.add_exclusive_option_group(title='Plot Type', options=self.plot_types, selected_index=self._plot_type_index)
        plot_type = dialog.get_settings()[0]
        if plot_type is not None and plot_type['selected_index'] is not None and self._plot_type_index != plot_type['selected_index']:
            self._switch_data_plot_widget(plot_type['selected_index'])

    def shutdown_plugin(self):
        self._widget.clean_up_subscribers()
