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

PKG = 'rxbag_plugins'
import roslib; roslib.load_manifest(PKG)
import rospy

import bisect
import csv
import sys
import time

import Image
import numpy
import pylab
import wx
import wx.lib.wxcairo

from rxbag import TopicMessageView

from chart import Chart
from plot_configure_frame import PlotConfigureFrame
from plot_data_loader     import PlotDataLoader
import image_helper

class PlotView(TopicMessageView):
    name = 'Plot'

    rows = [(   1, 'All'),
            (   2, 'Every 2nd message'),
            (   3, 'Every 3rd message'),
            (   4, 'Every 4th message'),
            (   5, 'Every 5th message'),
            (  10, 'Every 10th message'),
            (  20, 'Every 20th message'),
            (  50, 'Every 50th message'),
            ( 100, 'Every 100th message'),
            (1000, 'Every 1000th message')]

    def __init__(self, timeline, parent):
        TopicMessageView.__init__(self, timeline, parent)

        self._topic           = None
        self._message         = None
        self._plot_paths      = []
        self._playhead        = None
        self._chart           = Chart()
        self._data_loader     = None
        self._x_view          = None
        self._dirty_count     = 0
        self._csv_data_loader = None
        self._csv_path        = None
        self._csv_row_stride  = None

        self._clicked_pos = None
        self._dragged_pos = None

        self._configure_frame = None

        self._max_interval_pixels = 1.0

        tb = self.parent.ToolBar
        icons_dir = roslib.packages.get_pkg_dir(PKG) + '/icons/'
        tb.AddSeparator()
        tb.Bind(wx.EVT_TOOL, lambda e: self.configure(), tb.AddLabelTool(wx.ID_ANY, '', wx.Bitmap(icons_dir + 'cog.png')))

        self.parent.Bind(wx.EVT_SIZE,        self._on_size)
        self.parent.Bind(wx.EVT_PAINT,       self._on_paint)
        self.parent.Bind(wx.EVT_LEFT_DOWN,   self._on_left_down)
        self.parent.Bind(wx.EVT_MIDDLE_DOWN, self._on_middle_down)
        self.parent.Bind(wx.EVT_RIGHT_DOWN,  self._on_right_down)
        self.parent.Bind(wx.EVT_LEFT_UP,     self._on_left_up)
        self.parent.Bind(wx.EVT_MIDDLE_UP,   self._on_middle_up)
        self.parent.Bind(wx.EVT_RIGHT_UP,    self._on_right_up)
        self.parent.Bind(wx.EVT_MOTION,      self._on_mouse_move)
        self.parent.Bind(wx.EVT_MOUSEWHEEL,  self._on_mousewheel)
        self.parent.Bind(wx.EVT_CLOSE,       self._on_close)

        wx.CallAfter(self.configure)

    ## TopicMessageView implementation

    def message_viewed(self, bag, msg_details):
        TopicMessageView.message_viewed(self, bag, msg_details)

        topic, msg, t = msg_details

        if not self._data_loader:
            self._topic = topic
            self.start_loading()

        self._message = msg

        self.playhead = (t - self.timeline.start_stamp).to_sec()

    def message_cleared(self):
        self._message = None
        
        TopicMessageView.message_cleared(self)
        
        wx.CallAfter(self.parent.Refresh)

    def timeline_changed(self):
        # If timeline end_stamp is within the plot view, then invalidate the data loader
        if self._x_view is not None: 
            end_elapsed = (self.timeline.end_stamp - self.timeline.start_stamp).to_sec()
            if end_elapsed > self._x_view[0] and end_elapsed < self._x_view[1] and self._data_loader:
                self._data_loader.invalidate()

        wx.CallAfter(self.parent.Refresh)

    # property: plot_paths

    def _get_plot_paths(self): return self._plot_paths

    def _set_plot_paths(self, plot_paths):
        self._plot_paths = plot_paths

        # Update the data loader with the paths to plot
        if self._data_loader:
            paths = []
            for plot in self._plot_paths:
                for path in plot:
                    if path not in paths:
                        paths.append(path)

            self._data_loader.paths = paths

        # Update the chart with the new areas
        self._chart.create_areas(self._plot_paths)

        self._update_max_interval()
        
        wx.CallAfter(self.parent.Refresh)

    plot_paths = property(_get_plot_paths, _set_plot_paths)

    # property: playhead

    def _get_playhead(self): return self._playhead

    def _set_playhead(self, playhead):
        self._playhead = playhead
        
        # Check if playhead is visible. If not, then move the view region.
        if self._x_view is not None:
            if self._playhead < self._x_view[0]:
                x_view = self._x_view[1] - self._x_view[0]
                self._x_view = (self._playhead, self._playhead + x_view)
                self._update_data_loader_interval()

            elif self._playhead > self._x_view[1]:
                x_view = self._x_view[1] - self._x_view[0]
                self._x_view = (self._playhead - x_view, self._playhead)
                self._update_data_loader_interval()
        
        wx.CallAfter(self.parent.Refresh)

    playhead = property(_get_playhead, _set_playhead)

    def _update_max_interval(self):
        if not self._data_loader:
            return

        if len(self._chart.areas) > 0:
            secs_per_px = (self._data_loader.end_stamp - self._data_loader.start_stamp).to_sec() / self._chart._width
            self._data_loader.max_interval = secs_per_px * self._max_interval_pixels

    ## Events

    def _on_paint(self, event):
        if not self._data_loader or len(self._chart._areas) == 0:
            return

        self._update_chart_info()

        dc = wx.lib.wxcairo.ContextFromDC(wx.PaintDC(self.parent))

        self._chart.paint(dc)

    def _update_chart_info(self):
        for area_index, plot in enumerate(self._plot_paths):
            area = self._chart.areas[area_index]

            area.x_view = self._x_view
            
            if self._message is not None:
                area.x_indicator = self._playhead
            else:
                area.x_indicator = None

            area.x_range = (0.0, (self.timeline.end_stamp - self.timeline.start_stamp).to_sec())
            
            if self._data_loader.is_load_complete:
                area.data_alpha = 1.0
            else:
                area.data_alpha = 0.5

            area._series_list = plot

            data = {}
            for plot_path in plot:
                if plot_path in self._data_loader._data:
                    data[plot_path] = self._data_loader._data[plot_path]
            area._series_data = data

    def _on_size(self, event):
        self._chart.set_size(self.parent.ClientSize)

        self._update_max_interval()

    def _on_left_down(self, event):
        self._clicked_pos = self._dragged_pos = event.Position
        if len(self._chart.areas) > 0 and self._chart.areas[0].view_min_x is not None and self._chart.areas[0].view_max_x is not None:
            self.timeline.playhead = self.timeline.start_stamp + rospy.Duration.from_sec(max(0.01, self._chart.areas[0].x_chart_to_data(event.Position[0])))

    def _on_middle_down(self, event):
        self._clicked_pos = self._dragged_pos = event.Position

    def _on_right_down(self, event):
        self._clicked_pos = self._dragged_pos = event.Position
        self.parent.PopupMenu(PlotPopupMenu(self.parent, self), self._clicked_pos)

    def _on_left_up  (self, event): self._on_mouse_up(event)
    def _on_middle_up(self, event): self._on_mouse_up(event)
    def _on_right_up (self, event): self._on_mouse_up(event)

    def _on_mouse_up(self, event): self.parent.Cursor = wx.StockCursor(wx.CURSOR_ARROW)
    
    def _on_mouse_move(self, event):
        x, y = event.Position
        
        if event.Dragging():
            if event.MiddleIsDown() or event.ShiftDown():
                # Middle or shift: zoom
                
                dx_click, dy_click = x - self._clicked_pos[0], y - self._clicked_pos[1]
                dx_drag,  dy_drag  = x - self._dragged_pos[0], y - self._dragged_pos[1]

                if dx_drag != 0 and len(self._chart.areas) > 0:
                    dsecs = self._chart.areas[0].dx_chart_to_data(dx_drag)  # assuming areas share x axis
                    self._translate_plot(dsecs)

                if dy_drag != 0:
                    self._zoom_plot(1.0 + 0.005 * dy_drag)

                self._update_data_loader_interval()

                wx.CallAfter(self.parent.Refresh)

                self.parent.Cursor = wx.StockCursor(wx.CURSOR_HAND)

            elif event.LeftIsDown():
                if len(self._chart.areas) > 0 and self._chart.areas[0].view_min_x is not None and self._chart.areas[0].view_max_x is not None:
                    self.timeline.playhead = self.timeline.start_stamp + rospy.Duration.from_sec(max(0.01, self._chart.areas[0].x_chart_to_data(x)))

                wx.CallAfter(self.parent.Refresh)
            
            self._dragged_pos = event.Position

    def _on_mousewheel(self, event):
        dz = event.WheelRotation / event.WheelDelta
        self._zoom_plot(1.0 - dz * 0.2)

        self._update_data_loader_interval()

        wx.CallAfter(self.parent.Refresh)

    def _on_close(self, event):
        if self._configure_frame:
            self._configure_frame.Close()

        self.stop_loading()

        event.Skip()

    ##

    def _update_data_loader_interval(self):
        self._data_loader.set_interval(self.timeline.start_stamp + rospy.Duration.from_sec(max(0.01, self._x_view[0])),
                                       self.timeline.start_stamp + rospy.Duration.from_sec(max(0.01, self._x_view[1])))

    def _zoom_plot(self, zoom):
        if self._x_view is None:
            self._x_view = (0.0, (self.timeline.end_stamp - self.timeline.start_stamp).to_sec())

        x_view_interval = self._x_view[1] - self._x_view[0]
        if x_view_interval == 0.0:
            return

        playhead_fraction = (self._playhead - self._x_view[0]) / x_view_interval

        new_x_view_interval = zoom * x_view_interval

        # Enforce zoom limits (0.1s, 1.5 * range)
        max_zoom_interval = (self.timeline.end_stamp - self.timeline.start_stamp).to_sec() * 1.5
        if new_x_view_interval > max_zoom_interval:
            new_x_view_interval = max_zoom_interval
        elif new_x_view_interval < 0.1:
            new_x_view_interval = 0.1

        interval_0 = self._playhead - playhead_fraction * new_x_view_interval
        interval_1 = interval_0 + new_x_view_interval

        timeline_range = (self.timeline.end_stamp - self.timeline.start_stamp).to_sec()
        interval_0 = min(interval_0, timeline_range - 0.1)
        interval_1 = max(interval_1, 0.1)

        self._x_view = (interval_0, interval_1)

        self._update_max_interval()

    def _translate_plot(self, dsecs):
        if self._x_view is None:
            self._x_view = (0.0, (self.timeline.end_stamp - self.timeline.start_stamp).to_sec())

        new_start = self._x_view[0] - dsecs
        new_end   = self._x_view[1] - dsecs

        timeline_range = (self.timeline.end_stamp - self.timeline.start_stamp).to_sec()
        new_start = min(new_start, timeline_range - 0.1)
        new_end   = max(new_end,   0.1)

        self._x_view = (new_start, new_end)

    ##

    def stop_loading(self):
        if self._data_loader:
            self._data_loader.stop()
            self._data_loader = None

    def start_loading(self):
        if self._topic and not self._data_loader:
            self._data_loader = PlotDataLoader(self.timeline, self._topic)
            self._data_loader.add_progress_listener(self._data_loader_updated)
            self._data_loader.add_complete_listener(self._data_loader_complete)
            self._data_loader.start()

    def _data_loader_updated(self):
        self._dirty_count += 1
        if self._dirty_count > 5:
            wx.CallAfter(self.parent.Refresh)
            self._dirty_count = 0

    def _data_loader_complete(self):
        wx.CallAfter(self.parent.Refresh)

    def configure(self):
        if self._configure_frame is not None or self._message is None:
            return

        self._configure_frame = PlotConfigureFrame(self)

        frame = self.parent.TopLevelParent
        self._configure_frame.Position = (frame.Position[0] + frame.Size[0] + 10, frame.Position[1])
        self._configure_frame.Show()

    ## Export to CSV...

    def export_csv(self, rows):
        dialog = wx.FileDialog(self.parent.Parent, 'Export to CSV...', wildcard='CSV files (*.csv)|*.csv', style=wx.FD_SAVE)
        if dialog.ShowModal() == wx.ID_OK:
            if self.timeline.start_background_task('Exporting to "%s"' % dialog.Path):
                wx.CallAfter(wx.GetApp().GetTopWindow().StatusBar.gauge.Show)
                
                export_series = set()
                for plot in self._plot_paths:
                    for path in plot:
                        export_series.add(path)
    
                if self._x_view is None:
                    self._x_view = (0.0, (self.timeline.end_stamp - self.timeline.start_stamp).to_sec())
    
                self._csv_path       = dialog.Path
                self._csv_row_stride = rows
    
                self._csv_data_loader = PlotDataLoader(self.timeline, self._topic)
                self._csv_data_loader.add_complete_listener(self._csv_data_loaded)
                self._csv_data_loader.paths = export_series
                self._csv_data_loader.set_interval(self.timeline.start_stamp + rospy.Duration.from_sec(max(0.01, self._x_view[0])),
                                                   self.timeline.start_stamp + rospy.Duration.from_sec(max(0.01, self._x_view[1])))
                self._csv_data_loader.start()

        dialog.Destroy()

    def _csv_data_loaded(self):
        # Collate data
        i = 0
        series_dict = {}
        unique_stamps = set()
        for series in self._csv_data_loader._data:
            d = {}
            series_dict[series] = d
            point_num = 0
            for x, y in self._csv_data_loader._data[series].points:
                if point_num % self._csv_row_stride == 0:
                    d[x] = y
                    unique_stamps.add(x)
                point_num += 1
            i += 1
        series_columns = sorted(series_dict.keys())

        try:
            csv_writer = csv.DictWriter(open(self._csv_path, 'w'), ['elapsed'] + series_columns)
 
            # Write header row
            header_dict = { 'elapsed' : 'elapsed' }
            for column in series_columns:
                header_dict[column] = column            
            csv_writer.writerow(header_dict)

            # Initialize progress monitoring
            progress = 0
            def update_progress(v):
                wx.GetApp().TopWindow.StatusBar.progress = v
            total_stamps = len(unique_stamps)
            stamp_num = 0
            
            # Write data
            for stamp in sorted(unique_stamps):
                if self.timeline.background_task_cancel:
                    break
                
                row = { 'elapsed' : stamp }
                for column in series_dict:
                    if stamp in series_dict[column]:
                        row[column] = series_dict[column][stamp]

                csv_writer.writerow(row)

                new_progress = int(100.0 * (float(stamp_num) / total_stamps))
                if new_progress != progress:
                    progress = new_progress
                    wx.CallAfter(update_progress, progress)

                stamp_num += 1

        except Exception, ex:
            print >> sys.stderr, 'Error writing to CSV file: %s' % str(ex)

        # Hide progress monitoring
        if not self.timeline.background_task_cancel:
            wx.CallAfter(wx.GetApp().TopWindow.StatusBar.gauge.Hide)

        self.timeline.stop_background_task()

    ## Save plot to...
        
    def export_image(self):
        dialog = wx.FileDialog(self.parent.Parent, 'Save plot to...', wildcard='PNG files (*.png)|*.png', style=wx.FD_SAVE)
        if dialog.ShowModal() == wx.ID_OK:
            path = dialog.Path

            bitmap = wx.EmptyBitmap(self._chart._width, self._chart._height)
            mem_dc = wx.MemoryDC()
            mem_dc.SelectObject(bitmap)
            mem_dc.SetBackground(wx.WHITE_BRUSH)
            mem_dc.Clear()
            cairo_dc = wx.lib.wxcairo.ContextFromDC(mem_dc)
            self._chart.paint(cairo_dc)
            mem_dc.SelectObject(wx.NullBitmap)

            bitmap.SaveFile(path, wx.BITMAP_TYPE_PNG)
            
        dialog.Destroy()

class PlotPopupMenu(wx.Menu):
    def __init__(self, parent, plot):
        wx.Menu.__init__(self)

        self.parent = parent
        self.plot   = plot

        # Configure...
        configure_item = wx.MenuItem(self, wx.NewId(), 'Configure...')
        self.AppendItem(configure_item)
        self.Bind(wx.EVT_MENU, lambda e: self.plot.configure(), id=configure_item.Id)

        # Export to PNG...
        export_image_item = wx.MenuItem(self, wx.NewId(), 'Export to PNG...')
        self.AppendItem(export_image_item)
        self.Bind(wx.EVT_MENU, lambda e: self.plot.export_image(), id=export_image_item.Id)
        
        # Export to CSV...
        self.export_csv_menu = wx.Menu()
        self.AppendSubMenu(self.export_csv_menu, 'Export to CSV...', 'Export data to CSV file')

        for rows, label in plot.rows:
            rows_item = self.ExportCSVMenuItem(self.export_csv_menu, wx.NewId(), label, rows, plot)
            self.export_csv_menu.AppendItem(rows_item)
            
            if label == 'All':
                self.export_csv_menu.AppendSeparator()

    class ExportCSVMenuItem(wx.MenuItem):
        def __init__(self, parent, id, label, rows, plot):
            wx.MenuItem.__init__(self, parent, id, label)
            
            self.rows = rows
            self.plot = plot

            parent.Bind(wx.EVT_MENU, self._on_menu, id=self.Id)

        def _on_menu(self, event):
            self.plot.export_csv(self.rows)
