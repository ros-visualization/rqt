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
from QtCore import Qt
from QtGui import QFileDialog, QGraphicsView, QIcon, QWidget

import rosbag
from .bag_timeline import BagTimeline


class BagWidget(QWidget):
    """
    Widget for use with Bag class to display and replay bag files
    Handles all widget callbacks and contains the instance of BagTimeline for storing visualizing bag data
    """
    def __init__(self, context):
        """
        :param context: plugin context hook to enable adding widgets as a ROS_GUI pane, ''PluginContext''
        """
        super(BagWidget, self).__init__()
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'bag_widget.ui')
        loadUi(ui_file, self)
        context.add_widget(self)

        self.setObjectName('BagWidget')

        self._timeline = BagTimeline(context)
        self.graphics_view.setScene(self._timeline)

        self.graphics_view.resizeEvent = self._resizeEvent
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

        self.play_button.clicked[bool].connect(self._handle_play_clicked)
        self.thumbs_button.clicked[bool].connect(self._handle_thumbs_clicked)
        self.zoom_in_button.clicked[bool].connect(self._handle_zoom_in_clicked)
        self.zoom_out_button.clicked[bool].connect(self._handle_zoom_out_clicked)
        self.zoom_all_button.clicked[bool].connect(self._handle_zoom_all_clicked)
        self.faster_button.clicked[bool].connect(self._handle_faster_clicked)
        self.slower_button.clicked[bool].connect(self._handle_slower_clicked)
        self.begin_button.clicked[bool].connect(self._handle_begin_clicked)
        self.end_button.clicked[bool].connect(self._handle_end_clicked)
        self.load_button.clicked[bool].connect(self._handle_load_clicked)
        self.save_button.clicked[bool].connect(self._handle_save_clicked)
        self.graphics_view.mousePressEvent = self._timeline.on_mouse_down
        self.graphics_view.mouseReleaseEvent = self._timeline.on_mouse_up
        self.graphics_view.mouseMoveEvent = self._timeline.on_mouse_move
        self.graphics_view.wheelEvent = self._timeline.on_mousewheel
        self.closeEvent = self.handle_close
        self.keyPressEvent = self.on_key_press
        # TODO when the closeEvent is properly called by ROS_GUI implement that event instead of destroyed
        self.destroyed.connect(self.handle_destroy)
        
        self.graphics_view.keyPressEvent = self.graphics_view_on_key_press

    def graphics_view_on_key_press(self, event):
        key = event.key()
        if key in (Qt.Key_Left, Qt.Key_Right, Qt.Key_Up, Qt.Key_Down, Qt.Key_PageUp, Qt.Key_PageDown):
            # This causes the graphics view to ignore these keys so they can be caught byt the bag_widget keyPressEvent
            event.ignore()
        else:
            # Maintains functionality for all other keys QGraphicsView implements
            QGraphicsView.keyPressEvent(self.graphics_view, event)

    # callbacks for ui events
    def on_key_press(self, event):
        key = event.key()
        if key == Qt.Key_Space:
            self._timeline.toggle_play()
        elif key == Qt.Key_Home:
            self._timeline.navigate_start()
        elif key == Qt.Key_End:
            self._handle_end_clicked()
        elif key == Qt.Key_Plus or key == Qt.Key_Equal:
            self._handle_faster_clicked()
        elif key == Qt.Key_Minus:
            self._handle_slower_clicked()
        elif key == Qt.Key_Left:
            self._timeline.translate_timeline_left()
        elif key == Qt.Key_Right:
            self._timeline.translate_timeline_right()
        elif key == Qt.Key_Up or key == Qt.Key_PageUp:
            self._handle_zoom_in_clicked()
        elif key == Qt.Key_Down or key == Qt.Key_PageDown:
            self._handle_zoom_out_clicked()
#        elif key == Qt.Key_Pause: 
#            self._timeline.toggle_recording()

    def handle_destroy(self, args):
        self._timeline.handle_close()

    def handle_close(self, event):
        # TODO: Figure out why ROS_GUI is not calling closeEvent when a plugin is closed (cause of the "plugin windows stay open after closing main window" issue)
        # ON HOLD: pending redesign of ROS_GUI plugin close functionality
        self.shutdown_all()
        
        event.accept()
    
    def _resizeEvent(self, event):
        # TODO make this smarter. currently there will be no scrollbar even if the timeline extends beyond the viewable area
        self.graphics_view.scene().setSceneRect(0, 0, self.graphics_view.size().width() - 2, self.graphics_view.size().height() - 2)

    def _handle_publish_clicked(self, checked):
        self._timeline.set_publishing_state(checked)

    def _handle_play_clicked(self, checked):
        if checked:
            self.play_button.setIcon(self.pause_icon)
            self._timeline.navigate_play()
        else:
            self.play_button.setIcon(self.play_icon)
            self._timeline.navigate_stop()

    def _handle_faster_clicked(self):
        self._timeline.navigate_fastforward()

    def _handle_slower_clicked(self):
        self._timeline.navigate_rewind()

    def _handle_begin_clicked(self):
        self._timeline.navigate_start()

    def _handle_end_clicked(self):
        self._timeline.navigate_end()

    def _handle_thumbs_clicked(self, checked):
        self._timeline._timeline_frame.toggle_renderers()
        # TODO consider changing the icon when the button is down

    def _handle_zoom_all_clicked(self):
        self._timeline.reset_zoom()

    def _handle_zoom_out_clicked(self):
        self._timeline.zoom_out()

    def _handle_zoom_in_clicked(self):
        self._timeline.zoom_in()

    def _handle_load_clicked(self):
        filename = QFileDialog.getOpenFileName(self, self.tr('Load from File'), '.', self.tr('Bag files {.bag} (*.bag)'))
        if filename[0] != '':
            bag = rosbag.Bag(filename[0])
            self._timeline.add_bag(bag)

    def _handle_save_clicked(self):
        filename = QFileDialog.getSaveFileName(self, self.tr('Save selected region to file...'), '.', self.tr('Bag files {.bag} (*.bag)'))
        if filename[0] != '':
            self._timeline.copy_region_to_bag(filename[0])
    
    # Shutdown all members
    
    def shutdown_all(self):
        self._timeline.handle_close()
