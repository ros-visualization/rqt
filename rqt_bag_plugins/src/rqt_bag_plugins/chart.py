# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from __future__ import division

PKG = 'rxbag_plugins'
import roslib; roslib.load_manifest(PKG)

import bisect
import math
import sys
import threading
import time

import cairo
import wx
import wx.lib.wxcairo

from dataset import DataSet

class ChartWindow(wx.Window):
    def __init__(self, *args, **kwargs):
        wx.Window.__init__(self, *args, **kwargs)

        self.background_brush = wx.WHITE_BRUSH

        self._chart = Chart()

        self.Bind(wx.EVT_PAINT, self._on_paint)
        self.Bind(wx.EVT_SIZE,  self._on_size)

    def _on_size(self, event):
        self._chart.set_size(*self.ClientSize)
        self.Refresh()

    def _on_paint(self, event):
        window_dc = wx.PaintDC(self)
        window_dc.SetBackground(self.background_brush)
        window_dc.Clear()

        dc = wx.lib.wxcairo.ContextFromDC(window_dc)
        self._chart.paint(dc)

class Chart(object):
    def __init__(self):
        self._areas = []

        self._margin_left   = 10
        self._margin_right  = 10
        self._margin_top    =  8
        self._margin_bottom =  2

        self._width  = 100
        self._height = 100

    @property
    def areas(self): return self._areas

    def set_size(self, size):
        self._width, self._height = size

    def create_areas(self, plots):
        self._areas = []
        palette_offset = 0
        for i, plot in enumerate(plots):
            area = ChartArea()
            area.palette_offset = palette_offset
            palette_offset += len(plot)
            if i < len(plots) - 1:
                area.show_x_ticks = False
            self._areas.append(area)

    def paint(self, dc):
        # Clear background to white
        dc.set_source_rgb(1, 1, 1)
        dc.rectangle(0, 0, self._width, self._height)
        dc.fill()

        self._calculate_area_bounds(dc)

        # Paint areas
        dc.save()
        area_height = self._height / len(self._areas)
        for area in self._areas:
            area.paint(dc)
            dc.translate(0, area_height)
        dc.restore()

    def _calculate_area_bounds(self, dc):
        if len(self._areas) == 0:
            return
        
        # Calculate top, height
        new_bounds_top    = self._margin_top
        area_height = self._height / len(self._areas)
        new_bounds_height = area_height - new_bounds_top - self._margin_bottom
        for area in self._areas:
            area._width  = self._width
            area._height = area_height

            area._bounds_top = new_bounds_top

            if area.show_x_ticks:
                area._bounds_height = new_bounds_height - 18
            else:
                area._bounds_height = new_bounds_height

        # Calculate Y interval
        for area in self._areas:
            area._update_y_interval(dc)

        # Calculate left, width
        max_width = None
        for area in self._areas:
            # Calculate the maximum width taken up by the Y tick labels
            if area.num_points > 0 and area.view_min_y is not None and area.view_max_y is not None and area._y_interval is not None:
                dc.set_font_size(area._tick_font_size)
                for x0, y0, x1, y1 in area._generate_lines_y(area._round_min_to_interval(area.view_min_y, area._y_interval),
                                                             area._round_max_to_interval(area.view_max_y, area._y_interval),
                                                             area._y_interval,
                                                             0, 0):
                    s = area.format_y(area.y_chart_to_data(y0))
                    text_width = dc.text_extents(s)[2]
                    width = text_width + 3
    
                    if max_width is None or width > max_width:
                        max_width = width

        if max_width is not None:
            new_bounds_left = self._margin_left + max_width
        else:
            new_bounds_left = self._margin_left

        new_bounds_width  = self._width - new_bounds_left - self._margin_right

        for area in self._areas:
            area._bounds_left  = new_bounds_left
            area._bounds_width = new_bounds_width

        # Calculate X interval
        for area in self._areas:
            area._update_x_interval(dc)

class ChartArea(object):
    def __init__(self):
        self._lock = threading.RLock()

        ## Rendering info

        self._width  = 400
        self._height = 400

        self._palette = [(0.0, 0.0, 0.7),
                         (0.0, 0.7, 0.0),
                         (0.7, 0.0, 0.0),
                         (0.0, 0.7, 0.7),
                         (0.7, 0.0, 0.7),
                         (0.7, 0.7, 0.0)]

        self._tick_length        = 4
        self._tick_font_size     = 12.0
        self._tick_label_padding = 50

        self._legend_position       = ( 7.0, 6.0)   # pixel offset of top-left of legend from top-left of chart
        self._legend_margin         = ( 6.0, 3.0)   # internal legend margin  
        self._legend_line_thickness =  3.0          # thickness of series color indicator
        self._legend_line_width     =  9.0          # width of series color indicator
        self._legend_font_size      = 12.0          # height of series font
        self._legend_line_spacing   =  1.0          # spacing between lines on the legend

        self._palette_offset = 0
        self._show_lines     = True
        self._show_points    = True

        self._x_indicator_color     = (1.0, 0.2, 0.2, 0.8)
        self._x_indicator_thickness = 2.0
        
        ###

        self._x_interval   = None
        self._y_interval   = None
        self._show_x_ticks = True

        # The viewport
        self._x_view = None
        self._y_view = None
        
        self.x_indicator = None
        self.x_range     = None
        
        self.data_alpha = 1.0

        ##

        self._series_list = []    # [series0, ...]
        self._series_data = {}    # str -> DataSet

        self._bounds_left   = None
        self._bounds_top    = None
        self._bounds_width  = None
        self._bounds_height = None

    @property
    def num_points(self):
        num_points = 0
        for dataset in self._series_data.values():
            num_points += dataset.num_points
        return num_points

    @property
    def min_x(self):
        min_x = None
        for dataset in self._series_data.values():
            if dataset.num_points > 0:
                if min_x is None:
                    min_x = dataset.min_x
                else:
                    min_x = min(min_x, dataset.min_x)
        return min_x

    @property
    def max_x(self):
        max_x = None
        for dataset in self._series_data.values():
            if dataset.num_points > 0:
                if max_x is None:
                    max_x = dataset.max_x
                else:
                    max_x = max(max_x, dataset.max_x)
        return max_x

    @property
    def min_y(self):
        min_y = None
        for dataset in self._series_data.values():
            if dataset.num_points > 0:
                if min_y is None:
                    min_y = dataset.min_y
                else:
                    min_y = min(min_y, dataset.min_y)
        return min_y

    @property
    def max_y(self):
        max_y = None
        for dataset in self._series_data.values():
            if dataset.num_points > 0:
                if max_y is None:
                    max_y = dataset.max_y
                else:
                    max_y = max(max_y, dataset.max_y)
        return max_y

    # palette_offset
    
    def _get_palette_offset(self):
        return self._palette_offset

    def _set_palette_offset(self, palette_offset):
        self._palette_offset = palette_offset
        # @todo: refresh

    palette_offset = property(_get_palette_offset, _set_palette_offset)

    # show_x_ticks

    def _get_show_x_ticks(self):
        return self._show_x_ticks
    
    def _set_show_x_ticks(self, show_x_ticks):
        self._show_x_ticks = show_x_ticks

    show_x_ticks = property(_get_show_x_ticks, _set_show_x_ticks)

    # x_view
    
    def _get_x_view(self):
        if self._x_view is not None:
            return self._x_view

        if self.min_x == self.max_x and self.min_x is not None:
            return (self.min_x - 0.01, self.max_x + 0.01)
        else:
            return (self.min_x, self.max_x)

    def _set_x_view(self, x_view):
        self._x_view = x_view

    x_view = property(_get_x_view, _set_x_view)

    @property
    def y_view(self):
        if self._y_view is not None:
            return self._y_view

        if self.min_y == self.max_y and self.min_y is not None:
            return (self.min_y - 0.01, self.max_y + 0.01)
        else:
            return (self.min_y, self.max_y)

    @property
    def view_range_x(self): return self.view_max_x - self.view_min_x
    
    @property
    def view_min_x(self): return self.x_view[0]

    @property
    def view_max_x(self): return self.x_view[1]

    @property
    def view_range_y(self): return self.view_max_y - self.view_min_y

    @property
    def view_min_y(self): return self.y_view[0]

    @property
    def view_max_y(self): return self.y_view[1]

    ## Layout

    @property
    def bounds_left(self): return self._bounds_left
    
    @property
    def bounds_top(self): return self._bounds_top

    @property
    def bounds_width(self): return self._bounds_width
    
    @property
    def bounds_height(self): return self._bounds_height

    @property
    def bounds_right(self): return self._bounds_left + self._bounds_width

    @property
    def bounds_bottom(self): return self._bounds_top + self._bounds_height

    ## Data

    def add_datum(self, series, x, y):
        with self._lock:
            if series not in self._series_data:
                self._series_list.append(series)
                self._series_data[series] = DataSet()

            self._series_data[series].add(x, y)

    def clear(self):
        with self._lock:
            self._series_list = []
            self._series_data = {}

    ## Coordinate transformations

    def coord_data_to_chart(self, x, y): return self.x_data_to_chart(x), self.y_data_to_chart(y)
    def x_data_to_chart(self, x):        return self.bounds_left   + (x - self.view_min_x) / self.view_range_x * self.bounds_width
    def y_data_to_chart(self, y):        return self.bounds_bottom - (y - self.view_min_y) / self.view_range_y * self.bounds_height
    def dx_data_to_chart(self, dx):      return dx / self.view_range_x * self.bounds_width
    def dy_data_to_chart(self, dy):      return dy / self.view_range_y * self.bounds_height

    def x_chart_to_data(self, x):        return self.view_min_x + (float(x) - self.bounds_left)   / self.bounds_width  * self.view_range_x
    def y_chart_to_data(self, y):        return self.view_min_y + (self.bounds_bottom - float(y)) / self.bounds_height * self.view_range_y
    def dx_chart_to_data(self, dx):      return float(dx) / self.bounds_width  * self.view_range_x
    def dy_chart_to_data(self, dy):      return float(dy) / self.bounds_height * self.view_range_y
    
    ## Data formatting
    
    def format_x(self, x, x_interval=None):
        if x_interval is None:
            x_interval = self._x_interval
        
        if self._x_interval is None:
            return '%.3f' % x
        
        dp = max(0, int(math.ceil(-math.log10(x_interval))))
        if dp == 0:
            return self.format_group(x)
        else:
            return '%.*f' % (dp, x)
    
    def format_y(self, y, y_interval=None):
        if y_interval is None:
            y_interval = self._y_interval
        
        if y_interval is None:
            return '%.3f' % y

        dp = max(0, int(math.ceil(-math.log10(y_interval))))
        if dp == 0:
            return self.format_group(y)
        else:
            return '%.*f' % (dp, y)

    def format_group(self, number):
        s = '%d' % round(number)
        groups = []
        while s and s[-1].isdigit():
            groups.append(s[-3:])
            s = s[:-3]
            
        return s + ','.join(reversed(groups))

    ## Layout

    def _update_x_interval(self, dc):
        if self.num_points == 0:
            return

        num_ticks = None
        if self.view_min_x is not None and self.view_max_x is not None and self._x_interval is not None:
            dc.set_font_size(self._tick_font_size)
            for test_num_ticks in range(20, 1, -1):
                test_x_interval = self._get_axis_interval((self.x_view[1] - self.x_view[0]) / test_num_ticks)
                max_width = self._get_max_label_width(dc, test_x_interval)
                
                max_width += 50   # add padding
                
                if max_width * test_num_ticks < self.bounds_width:
                    num_ticks = test_num_ticks
                    break
        
        if num_ticks is None:
            num_ticks = self.bounds_width / 100

        new_x_interval = self._get_axis_interval((self.x_view[1] - self.x_view[0]) / num_ticks)
        if new_x_interval is not None:
        	self._x_interval = new_x_interval 

    def _get_max_label_width(self, dc, x_interval):
        max_width = None

        for x0, y0, x1, y1 in self._generate_lines_x(self._round_min_to_interval(self.view_min_x, x_interval),
                                                     self._round_max_to_interval(self.view_max_x, x_interval),
                                                     x_interval,
                                                     self.bounds_bottom,
                                                     self.bounds_bottom + self._tick_length):
            s = self.format_x(self.x_chart_to_data(x0), x_interval)
            text_width = dc.text_extents(s)[2]
            width = text_width

            if max_width is None or width > max_width:
                max_width = width 

        return max_width

    def _update_y_interval(self, dc):
        if self.num_points == 0:
            return
        
        dc.set_font_size(self._tick_font_size)

        label_height = dc.font_extents()[2] + self._tick_label_padding

        num_ticks = self.bounds_height / label_height

        new_y_interval = self._get_axis_interval((self.y_view[1] - self.y_view[0]) / num_ticks)
        if new_y_interval is not None:
        	self._y_interval = new_y_interval

    def _get_axis_interval(self, range, intervals=[1.0, 2.0, 5.0]):
        exp = -8
        found = False
        prev_threshold = None
        while True:
            multiplier = pow(10, exp)
            for interval in intervals:
                threshold = multiplier * interval
                if threshold > range:
                    return prev_threshold
                prev_threshold = threshold
            exp += 1

    def _round_min_to_interval(self, min_val, interval, extend_touching=False):
        rounded = interval * math.floor(min_val / interval)
        if min_val > rounded:
            return rounded
        if extend_touching:
            return min_val - interval
        return min_val

    def _round_max_to_interval(self, max_val, interval, extend_touching=False):
        rounded = interval * math.ceil(max_val / interval)
        if max_val < rounded:
            return rounded
        if extend_touching:
            return max_val + interval
        return max_val

    ## Painting

    def paint(self, dc):
        self._draw_border(dc)

        dc.save()
        dc.rectangle(self.bounds_left, self.bounds_top, self.bounds_width, self.bounds_height)
        dc.clip()

        try:
            with self._lock:
                self._draw_data_extents(dc)
                self._draw_grid(dc)
                self._draw_axes(dc)
                self._draw_data(dc)
                self._draw_x_indicator(dc)
                self._draw_legend(dc)

        finally:
            dc.restore()

        self._draw_ticks(dc)

    def _draw_border(self, dc):
        dc.set_antialias(cairo.ANTIALIAS_NONE)
        dc.set_line_width(1.0)
        dc.set_source_rgba(0, 0, 0, 0.6)
        dc.rectangle(self.bounds_left, self.bounds_top - 1, self.bounds_width, self.bounds_height + 1)
        dc.stroke()

    def _draw_data_extents(self, dc):
        dc.set_source_rgba(0.5, 0.5, 0.5, 0.1)
    	if self.num_points == 0:
    		dc.rectangle(self.bounds_left, self.bounds_top, self.bounds_width, self.bounds_height)
    	else:
    		x_start, x_end = self.x_data_to_chart(self.min_x), self.x_data_to_chart(self.max_x)
    		dc.rectangle(self.bounds_left, self.bounds_top, x_start - self.bounds_left,                   self.bounds_height)
    		dc.rectangle(x_end,            self.bounds_top, self.bounds_left + self.bounds_width - x_end, self.bounds_height)
        dc.fill()

        if self.x_range is not None and self.num_points > 0:
            x_range_start, x_range_end = self.x_data_to_chart(self.x_range[0]), self.x_data_to_chart(self.x_range[1])
            dc.set_source_rgba(0.2, 0.2, 0.2, 0.1)
            dc.rectangle(self.bounds_left, self.bounds_top, x_range_start - self.bounds_left,                   self.bounds_height)
            dc.rectangle(x_range_end,      self.bounds_top, self.bounds_left + self.bounds_width - x_range_end, self.bounds_height)
            dc.fill()

    def _draw_grid(self, dc):
        dc.set_antialias(cairo.ANTIALIAS_NONE)
        dc.set_line_width(1.0)
        dc.set_dash([2, 4])

        if self.view_min_x != self.view_max_x and self._x_interval is not None:
            dc.set_source_rgba(0, 0, 0, 0.4)
            x_tick_range = (self._round_min_to_interval(self.view_min_x, self._x_interval),
                            self._round_max_to_interval(self.view_max_x, self._x_interval))
            self._draw_lines(dc, self._generate_lines_x(x_tick_range[0], x_tick_range[1], self._x_interval))

        if self.view_min_y != self.view_max_y and self._y_interval is not None:
            dc.set_source_rgba(0, 0, 0, 0.4)
            y_tick_range = (self._round_min_to_interval(self.view_min_y, self._y_interval),
                            self._round_max_to_interval(self.view_max_y, self._y_interval))
            self._draw_lines(dc, self._generate_lines_y(y_tick_range[0], y_tick_range[1], self._y_interval))

        dc.set_dash([])

    def _draw_axes(self, dc):
        dc.set_antialias(cairo.ANTIALIAS_NONE)
        dc.set_line_width(1.0)
        dc.set_source_rgba(0, 0, 0, 0.3)
        
        if self.view_min_y != self.view_max_y:
            x_intercept = self.y_data_to_chart(0.0)
            dc.move_to(self.bounds_left,  x_intercept)
            dc.line_to(self.bounds_right, x_intercept)
            dc.stroke()
        
        if self.view_min_x != self.view_max_x:
            y_intercept = self.x_data_to_chart(0.0)
            dc.move_to(y_intercept, self.bounds_bottom)
            dc.line_to(y_intercept, self.bounds_top)
            dc.stroke()

    def _draw_ticks(self, dc):
        dc.set_antialias(cairo.ANTIALIAS_NONE)
        dc.set_line_width(1.0)
        
        dc.set_font_size(self._tick_font_size)

        # Draw X axis ticks
        if self._show_x_ticks and self.view_min_x != self.view_max_x and self._x_interval is not None:
            x_tick_range = (self._round_min_to_interval(self.view_min_x, self._x_interval),
                            self._round_max_to_interval(self.view_max_x, self._x_interval))
            
            lines = list(self._generate_lines_x(x_tick_range[0], x_tick_range[1], self._x_interval, self.bounds_bottom, self.bounds_bottom + self._tick_length))

            dc.set_source_rgba(0, 0, 0, 1)
            self._draw_lines(dc, lines)

            for x0, y0, x1, y1 in lines:
                s = self.format_x(self.x_chart_to_data(x0))
                text_width, text_height = dc.text_extents(s)[2:4]

                tick_x = x0 - text_width / 2
                tick_y = y1 + 3 + text_height

                # Only show label if it's not outside the chart bounds
                if tick_x + text_width < self._width - 2:
                    dc.move_to(tick_x, tick_y)
                    dc.show_text(s)

        # Draw Y axis ticks
        if self.view_min_y != self.view_max_y and self._y_interval is not None:
            y_tick_range = (self._round_min_to_interval(self.view_min_y, self._y_interval),
                            self._round_max_to_interval(self.view_max_y, self._y_interval))

            lines = list(self._generate_lines_y(y_tick_range[0], y_tick_range[1], self._y_interval, self.bounds_left - self._tick_length, self.bounds_left))

            dc.set_source_rgba(0, 0, 0, 1)
            self._draw_lines(dc, lines)

            for x0, y0, x1, y1 in lines:
                s = self.format_y(self.y_chart_to_data(y0))
                text_width, text_height = dc.text_extents(s)[2:4]

                tick_x = x0 - text_width - 3
                tick_y = y0 + text_height / 2

                dc.move_to(tick_x, tick_y)
                dc.show_text(s)

    def _draw_lines(self, dc, lines):
        for x0, y0, x1, y1 in lines:
            dc.move_to(x0, y0)
            dc.line_to(x1, y1)
        dc.stroke()

    def _generate_lines_x(self, x0, x1, x_step, py0=None, py1=None):
        px0 = self.x_data_to_chart(x0)
        px1 = self.x_data_to_chart(x1)
        if py0 is None:
            py0 = self.bounds_bottom
        if py1 is None:
            py1 = self.bounds_top
        px_step = self.dx_data_to_chart(x_step)

        px = px0
        while True:
            if px >= self.bounds_left and px <= self.bounds_right:
                yield px, py0, px, py1
            px += px_step
            if px > px1:
                break

    def _generate_lines_y(self, y0, y1, y_step, px0=None, px1=None):
        py0 = self.y_data_to_chart(y0)
        py1 = self.y_data_to_chart(y1)
        if px0 is None:
            px0 = self.bounds_left
        if px1 is None:
            px1 = self.bounds_right
        py_step = self.dy_data_to_chart(y_step)
        
        py = py0
        while True:
            if py >= self.bounds_top and py <= self.bounds_bottom:
                yield px0, py, px1, py
            py -= py_step
            if py < py1:
                break

    def _draw_data(self, dc):
        if len(self._series_data) == 0:
            return

        for series, series_data in self._series_data.items():
            dc.set_source_rgba(*self._get_color(series))

            coords = [self.coord_data_to_chart(x, y) for x, y in series_data.points]

            # Only display points with the chart bounds (or 1 off)
            filtered_coords = []
            x_min, x_max = self.bounds_left, self.bounds_right
            for i, (x, y) in enumerate(coords):
                if x < x_min:
                    if i < len(coords) - 1 and coords[i + 1][0] >= x_min:
                        filtered_coords.append((x, y))
                elif x > x_max:
                    if i > 0 and coords[i - 1][0] <= x_max:
                        filtered_coords.append((x, y))
                else:
                    filtered_coords.append((x, y))
            coords = filtered_coords

            if len(coords) == 0:
                continue

            # Draw lines
            dc.set_antialias(cairo.ANTIALIAS_SUBPIXEL)
            if self._show_lines:
                dc.set_line_width(1.0)
                dc.move_to(*coords[0])
                for px, py in coords:
                    dc.line_to(px, py)
                dc.stroke()

            # Draw points
            dc.set_antialias(cairo.ANTIALIAS_NONE)
            if self._show_points:
                drawn_points = []
                last_x, last_y = -1, -1
                for px, py in coords:
                    if abs(px - last_x) >= 3 or abs(py - last_y) >= 3:
                        drawn_points.append((px, py))
                    last_x, last_y = px, py

                if len(drawn_points) > 0:
                    dc.set_line_width(1.0)
                    dc.set_source_rgb(1, 1, 1)
                    for px, py in drawn_points:
                        dc.rectangle(px - 1, py - 1, 2, 2)
                    dc.fill()
    
                    dc.set_source_rgba(*self._get_color(series))
                    for px, py in drawn_points:
                        dc.rectangle(px - 1, py - 1, 2, 2)
                    dc.stroke()

        dc.set_antialias(cairo.ANTIALIAS_SUBPIXEL)

    def _draw_x_indicator(self, dc):
        if self.x_indicator is None or self.view_min_x is None:
            return
        
        dc.set_antialias(cairo.ANTIALIAS_NONE)

        dc.set_line_width(self._x_indicator_thickness)
        dc.set_source_rgba(*self._x_indicator_color)

        px = self.x_data_to_chart(self.x_indicator)
        
        dc.move_to(px, self.bounds_top)
        dc.line_to(px, self.bounds_bottom)
        dc.stroke()

    def _draw_legend(self, dc):
    	if len(self._series_list) == 0:
    		return
    	
        dc.set_antialias(cairo.ANTIALIAS_NONE)

        dc.set_font_size(self._legend_font_size)
        font_height = dc.font_extents()[2]

        dc.save()
        
        dc.translate(self.bounds_left + self._legend_position[0], self.bounds_top + self._legend_position[1])
        
        legend_width = 0.0
        for series in self._series_list:
            legend_width = max(legend_width, self._legend_margin[0] + self._legend_line_width + 3.0 + dc.text_extents(series)[2] + self._legend_margin[0])

        legend_height = self._legend_margin[1] + (font_height * len(self._series_list)) + (self._legend_line_spacing * (len(self._series_list) - 1)) + self._legend_margin[1]

        dc.set_source_rgba(0.95, 0.95, 0.95, 0.5)
        dc.rectangle(0, 0, legend_width, legend_height)
        dc.fill()
        dc.set_source_rgba(0, 0, 0, 0.2)
        dc.set_line_width(1.0)
        dc.rectangle(0, 0, legend_width, legend_height)
        dc.stroke()
        
        # Drop shadow
        dc.set_source_rgba(0.5, 0.5, 0.5, 0.1)
        dc.move_to(1, legend_height + 1)
        dc.line_to(legend_width + 1, legend_height + 1)
        dc.line_to(legend_width + 1, 1)
        dc.stroke()

        dc.set_line_width(self._legend_line_thickness)

        dc.translate(self._legend_margin[0], self._legend_margin[1])

        for series in self._series_list:
            dc.set_source_rgba(*self._get_color(series, alpha=1.0))

            dc.move_to(0, font_height / 2)
            dc.line_to(self._legend_line_width, font_height / 2)
            dc.stroke()

            dc.translate(0, font_height)
            dc.move_to(self._legend_line_width + 3.0, -3)
            dc.show_text(series)

            dc.translate(0, self._legend_line_spacing)

        dc.restore()

    def _get_color(self, series, alpha=None):
        index = (self._palette_offset + self._series_list.index(series)) % len(self._palette)
        r, g, b = self._palette[index]
        if alpha is None:
            return (r, g, b, self.data_alpha)
        else:
            return (r, g, b, alpha)
