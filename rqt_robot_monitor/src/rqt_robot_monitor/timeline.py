# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
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
#
# Author: Isaac Saito, Ze'ev Klapow

import roslib;roslib.load_manifest('rqt_robot_monitor')
import rospy

from python_qt_binding import loadUi
from python_qt_binding.QtCore import QPointF, Qt, QRect, QSize, Signal
from python_qt_binding.QtGui import QBrush, QColor, QGraphicsPixmapItem, QGraphicsView, QIcon, QPainter, QPen, QWidget

class TimelineView(QGraphicsView):
    """
    When you instantiate this class, do NOT forget to call _set_init_data to 
    set necessary data.
    """
    
    _sig_update = Signal()
    #_sig_on_marker = Signal()
    
    def __init__(self, parent):
        """Cannot take args other than parent due to loadUi limitation."""
        
        super(TimelineView, self).__init__()
        self._parent = parent
        self._timeline_marker = QIcon.fromTheme('system-search')
        
        self._min_num_seconds = 0
        self._max_num_seconds = 0
        self._xpos_marker = 0
        self._color_callback = None
        
        self._timeline_marker_width = 20
        self._timeline_marker_height = 20
        
        self._sig_update.connect(self._slot_redraw)
        # self._color_callback = parent._color_callback
        #self._sig_on_marker.connect(parent.on_slider_scroll)
        
        self.setUpdatesEnabled(True) #In a trial to enable update()

    def _set_init_data(self, min_xpos_marker, max_num_seconds, 
                       xpos_marker, color_callback):
        """
        This function needs to be called right after the class is instantiated,
        in order to pass necessary values. _color_callback
        
        This function is to compensate the functional limitation of 
        python_qt_binding.loadUi that doesn't allow you to pass arguments in 
        the custom classes you use in .ui.
        
        
        @author: Isaac Saito 
        """
        self._min_num_seconds = min_xpos_marker
        self._max_num_seconds = max_num_seconds
        self._xpos_marker = xpos_marker
        self._color_callback = color_callback

    #def paintEvent(self, event): 
    ## Enabling this will collide with QGraphicsScene and ends up 
    ## with QGraphicsScene not being updated.
        # rospy.logdebug('test')
        # self.parent._slot_redraw()
        
    def redraw(self, event):
        """
        
        Draw timeline's "cells" (ie. boxes that represent seconds). 
                
        Taken from robot_monitor.message_timeline.on_paint
        
        @deprecated: This way (using QPainter) didn't work as intended.
        """       
        
        #super(TimelineView, self).paintEvent(event) 


        # painter = QPainter(self) 
        ## This yields warning that says:
        ## "QPainter::begin: Widget painting can only begin as a result of a paintEvent"

        painter = QPainter()
        painter.begin(self.viewport())
        pen = QPen(Qt.black, 2)#, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin)  # Needs modified.
        painter.setPen(pen)
                
        is_enabled = self.isEnabled()

        #(width_tl, height_tl) = self.size()
        qsize = self.size()
        width_tl = qsize.width()
        height_tl = qsize.height()
        
        length_tl = self._max_num_seconds + 1 - self._min_num_seconds
        rospy.logdebug('paintEvent is_enabled=%s length_tl=%d', 
                      is_enabled, length_tl)
        value_size = width_tl / float(length_tl)
        for i in xrange(0, length_tl):
            brush = None
            color_index = i + self._min_num_seconds
            if (is_enabled):
                qcolor = self._color_callback(color_index)                
            else:
                qcolor = QColor('grey')                
            end_color = QColor(0.5 * QColor('red').value(),
                               0.5 * QColor('green').value(),
                               0.5 * QColor('blue').value())
            #painter.setBrush(qcolor)
            painter.setBrush(QBrush(qcolor))
            start = i * value_size
            end = (i + 1) * value_size
            rect = QRect(start, 0, end, height_tl)
            
            # dc.GradientFillLinear(wx.Rect(start, 0, end, height_tl),
            #                       qcolor, end_color, wx.SOUTH)
            rospy.logdebug('paintEvent i=%d start=%s end=%s height_tl=%s',
                          i, start, end, height_tl)
            #painter.fillRect(QRect(start, end, 100, height_tl), qcolor)
            painter.fillRect(rect, qcolor)
            
            if (i > 0):
                # dc.SetPen(wx.BLACK_PEN)
                pen.setColor(Qt.black)
                # dc.DrawLine(start, 0, start, height_tl)
                painter.drawLine(start, 0, start, height_tl)
               
        size_marker = QSize(20, 20)
        marker_x = ((self._xpos_marker - 1) * value_size + 
                    (value_size / 2.0) - 
                    (self._timeline_marker.actualSize(size_marker).width() 
                     / 2.0))
        
        # dc.DrawBitmap(self._timeline_marker, marker_x, 0, True)
        qrect = QRect(marker_x, 0, size_marker.width(), size_marker.height())
        self._timeline_marker.paint(painter, qrect)
        rospy.logdebug(' paintEvent marker_x=%s', marker_x)           
        
        painter.end()

#    def resizeEvent(self, event):
#        self.adjustSize()  # TODO Make sure this meets my requirement.

    def _set_range(self, min_val, max_val):
        """
        
        Copied from robot_monitor
        
        @param min_val: Smallest second on timeline. 
        @param max_val: Largest second on timeline.         
        """
        self._min_num_seconds = min_val
        self._max_num_seconds = max_val
        rospy.logdebug(' TimelineView _set_range _min_num_seconds=%s max=%s',
                       self._min_num_seconds,
                       self._max_num_seconds)
#        self._xpos_marker = self.clamp(self._xpos_marker, self._min_num_seconds, 
#                                       self._max_num_seconds)
#        #self.Refresh()
#        #self.repaint() #Emits warning "QPixmap: It is not safe to use pixmaps 
#                        #outside the GUI thread"
#        self._sig_update.emit()
        self._set_xpos_marker(self._xpos_marker)

    def _set_xpos_marker(self, len_queue):
        """Copied from robot_monitor"""

        self._xpos_marker = self.clamp(int(len_queue),
                                       self._min_num_seconds, 
                                       self._max_num_seconds)
        rospy.logdebug(' TimelineView _set_xpos_marker len_queue=%s _xpos_marker=%s', 
                      len_queue, self._xpos_marker)
        #self._sig_update.emit()
        
    def _get_xpos_marker(self):
        return self._xpos_marker        
                
    def mouseReleaseEvent(self, event):
        '''
        Override.
        @param event: QMouseEvent
        '''

        self._parent.mouse_release(event)
        self._set_val_from_x(event.pos().x())
        
        # TODO Figure out what's done by this in wx.
        # wx.PostEvent(self.GetEventHandler(),
        #             wx.PyCommandEvent(wx.EVT_COMMAND_SCROLL_CHANGED.typeId,
        #                               self.GetId()))

    '''
    Override.
    @param event: QMouseEvent 
    '''        
    def mousePressEvent(self, evt):
        self._set_val_from_x(evt.pos().x())
                
        # TODO Figure out what's done by this in wx.
        # wx.PostEvent(self.GetEventHandler(),
        #             wx.PyCommandEvent(wx.EVT_COMMAND_SCROLL_THUMBTRACK.typeId,
        #                               self.GetId()))
        #self._sig_on_marker.emit()
        self._parent.on_slider_scroll(evt)
        
    def _set_val_from_x(self, x):
        '''
        Called when marker is moved by user.
        
        @param x: Position relative to self widget.
          
        Copied from robot_monitor.
        '''
        qsize = self.size()
        width = qsize.width()
        height = qsize.height()
        # determine value from mouse click
        length_tl_in_second = self._max_num_seconds + 1 - self._min_num_seconds
        width_cell = width / float(length_tl_in_second)
        x_marker_float = x / width_cell + 1
        self.set_marker_pos(x_marker_float)
        rospy.logdebug('TimelineView _set_val_from_x x=%s width_cell=%s ' +
                      'length_tl_in_second=%s set_marker_pos=%s', 
                      x, width_cell, length_tl_in_second, self._xpos_marker)

    def set_marker_pos(self, val):
        '''
        Copied from robot_monitor. Originally called SetValue.
        '''
        self._xpos_marker = self.clamp(int(val),
                                       self._min_num_seconds,
                                       self._max_num_seconds)
        #self.repaint()  # # This might result in segfault.
        self._sig_update.emit()  
                                  
           
    def clamp(self, val, min, max):
        """
        Judge if val is within the range given by min & max.
        If not, return either min or max.
        
        @param val: any number format
        @param min: any number format
        @param max: any number format
        @return: int
        
        Copied from robot_monitor.
        """        
        rospy.logdebug(' TimelineView clamp val=%s min=%s max=%s', 
                       val, min, max)
        if (val < min):
            return min
        if (val > max):
            return max
        return val
                
    def _slot_redraw(self):
        """
        Gets called either when new msg comes in or when marker is moved by user.
        """
        
        self._parent._scene.clear()

        is_enabled = self.isEnabled()
        
        qsize = self.size()
        width_tl = qsize.width()
        height_tl = qsize.height()
        
        length_tl = ((self._max_num_seconds + 1) - 
                     self._min_num_seconds)

        width_cell = width_tl / float(length_tl)
        len_queue = length_tl
 
        for i, m in enumerate(self._parent._get_diagnostic_queue()):
            w = float(width_tl / len_queue)
            h = self.viewport().height()
            
            # Figure out each cell's color.
            qcolor = None
            color_index = i + self._min_num_seconds
            if (is_enabled):
                qcolor = self._color_callback(self._parent._get_diagnostic_queue(),
                                              color_index)                
            else:
                qcolor = QColor('grey')
            
            # For adding gradation to the cell color. Not used yet.
            end_color = QColor(0.5 * QColor('red').value(),
                               0.5 * QColor('green').value(),
                               0.5 * QColor('blue').value())
                        
            rect = self._parent._scene.addRect(w * i, 0, w, h, 
                                               QColor('white'), qcolor)            
        
        # Setting marker.
        xpos_marker = ((self._xpos_marker - 1) * width_cell + 
                       (width_cell / 2.0) - 
                       (self._timeline_marker_width / 2.0))
        pos_marker = QPointF(xpos_marker, 0)
                
        # Need to instantiate marker everytime since it gets deleted 
        # in every loop by scene.clear()
        timeline_marker = self.instantiate_tl_icon()
        timeline_marker.setPos(pos_marker)
        self._parent._scene.addItem(timeline_marker)
        # self._timeline_marker.paint(painter, qrect)
        rospy.logdebug(' _slot_redraw xpos_marker=%s self._timeline_marker=%s ' + 
                      'pos_marker=%s',
                      xpos_marker, timeline_marker, pos_marker)

    def instantiate_tl_icon(self):
        timeline_marker_icon = QIcon.fromTheme('system-search')
        timeline_marker_icon_pixmap = timeline_marker_icon.pixmap(
                                                self._timeline_marker_width,
                                                self._timeline_marker_height)
        return QGraphicsPixmapItem(timeline_marker_icon_pixmap)     
  