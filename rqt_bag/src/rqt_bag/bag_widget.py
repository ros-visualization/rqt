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

import os

from qt_gui.qt_binding_helper import loadUi
#from QtCore import qWarning
from QtGui import QFileDialog, QIcon, QWidget

import rosbag
from .bag_timeline import BagTimeline


class BagWidget(QWidget):
    """
    Widget for use with Bag class to display and replay bag files
    """
    def __init__(self, context):
        super(BagWidget, self).__init__()
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'bag_widget.ui')
        loadUi(ui_file, self)
        context.add_widget(self)

        self.setObjectName('BagWidget')

        self._timeline = BagTimeline(self.graphics_view, context)
        self.graphics_view.setScene(self._timeline)

        self.graphics_view.resizeEvent = self.resizeEvent
        self.graphics_view.setMouseTracking(True)

        self.play_icon = QIcon.fromTheme('media-playback-start')
        self.pause_icon = QIcon.fromTheme('media-playback-pause')
        self.play_button.setIcon(self.play_icon)
        self.begin_button.setIcon(QIcon.fromTheme('media-skip-backward'))
        self.end_button.setIcon(QIcon.fromTheme('media-skip-forward'))
        self.slower_button.setIcon(QIcon.fromTheme('media-seek-backward'))
        self.faster_button.setIcon(QIcon.fromTheme('media-seek-forward'))
        self.zoom_in_button.setIcon(QIcon.fromTheme('zoom-in'))
        self.zoom_out_button.setIcon(QIcon.fromTheme('zoom-out'))
        self.zoom_all_button.setIcon(QIcon.fromTheme('zoom-original'))
        self.thumbs_button.setIcon(QIcon.fromTheme('insert-image'))
        self.load_button.setIcon(QIcon.fromTheme('document-open'))
        self.save_button.setIcon(QIcon.fromTheme('document-save'))

        self.play_button.clicked[bool].connect(self.handle_play_clicked)
        self.thumbs_button.clicked[bool].connect(self.handle_thumbs_clicked)
        self.zoom_in_button.clicked[bool].connect(self.handle_zoom_in_clicked)
        self.zoom_out_button.clicked[bool].connect(self.handle_zoom_out_clicked)
        self.zoom_all_button.clicked[bool].connect(self.handle_zoom_all_clicked)
        self.faster_button.clicked[bool].connect(self.handle_faster_clicked)
        self.slower_button.clicked[bool].connect(self.handle_slower_clicked)
        self.begin_button.clicked[bool].connect(self.handle_begin_clicked)
        self.end_button.clicked[bool].connect(self.handle_end_clicked)
        self.load_button.clicked[bool].connect(self.handle_load_clicked)
        self.save_button.clicked[bool].connect(self.handle_save_clicked)
#        self.publish_checkbox.clicked[bool].connect(self.handle_publish_clicked)
#        self.confine_checkbox.clicked[bool].connect(self.handle_confine_playhead_clicked)
        self.graphics_view.mousePressEvent = self._timeline.on_mouse_down
        self.graphics_view.mouseReleaseEvent = self._timeline.on_mouse_up
        self.graphics_view.mouseMoveEvent = self._timeline.on_mouse_move
        self.graphics_view.wheelEvent = self._timeline.on_mousewheel
        self.closeEvent = self._close  # TODO fix these items that are nolonger private

        #TODO verify we have implemented all the old keybindings from rxbag
#        self.Bind(wx.EVT_IDLE,        self.on_idle)
#        self.Bind(wx.EVT_PAINT,       self.on_paint)
#        self.Bind(wx.EVT_KEY_DOWN,    self.on_key_down)
#        self.Bind(wx.EVT_LEFT_DOWN,   self.on_left_down)
#        self.Bind(wx.EVT_MIDDLE_DOWN, self.on_middle_down)
#        self.Bind(wx.EVT_RIGHT_DOWN,  self.on_right_down)
#        self.Bind(wx.EVT_LEFT_UP,     self.on_left_up)
#        self.Bind(wx.EVT_MIDDLE_UP,   self.on_middle_up)
#        self.Bind(wx.EVT_RIGHT_UP,    self.on_right_up)
#        self.Bind(wx.EVT_MOTION,      self.on_mouse_move)
#        self.Bind(wx.EVT_MOUSEWHEEL,  self.on_mousewheel)

    def _close(self, event):
        self._timeline._close()
        event.accept()

    def resizeEvent(self, event):
        self.graphics_view.scene().setSceneRect(0, 0, self.graphics_view.size().width() - 2, self.graphics_view.size().height() - 2)

#    def handle_confine_playhead_clicked(self, checked):
#        self._timeline.set_confine_playhead_state(checked)

    def handle_publish_clicked(self, checked):
        self._timeline.set_publishing_state(checked)

    def handle_play_clicked(self, checked):
        if checked:
            self.play_button.setIcon(self.pause_icon)
            self._timeline.navigate_play()
        else:
            self.play_button.setIcon(self.play_icon)
            self._timeline.navigate_stop()

    def handle_faster_clicked(self):
        self._timeline.navigate_fastforward()

    def handle_slower_clicked(self):
        self._timeline.navigate_rewind()

    def handle_begin_clicked(self):
        self._timeline.navigate_start()

    def handle_end_clicked(self):
        self._timeline.navigate_end()

    def handle_thumbs_clicked(self, checked):
        # TODO consider changing the icon when the button is down
        pass

    def handle_zoom_all_clicked(self):
        self._timeline.reset_zoom()

    def handle_zoom_out_clicked(self):
        self._timeline.zoom_out()

    def handle_zoom_in_clicked(self):
        self._timeline.zoom_in()

    def handle_load_clicked(self):
        filename = QFileDialog.getOpenFileName(self, self.tr('Load from File'), '.', self.tr('Bag files {.bag} (*.bag)'))
        if filename[0] != '':
            bag = rosbag.Bag(filename[0])
            self._timeline.add_bag(bag)

    def handle_save_clicked(self):
        filename = QFileDialog.getSaveFileName(self, self.tr('Save selected region to file...'), '.', self.tr('Bag files {.bag} (*.bag)'))
        if filename[0] != '':
            self._timeline.copy_region_to_bag(filename[0])
