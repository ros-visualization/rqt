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

PKG = 'rqt_bag_plugins'
import roslib; roslib.load_manifest(PKG)

import os
import shutil
import sys
import threading
import time

import numpy

import Image
import ImageQt

from rqt_bag import bag_helper, TopicMessageView
import image_helper

from QtCore import Qt
from QtGui import QBrush, QFileDialog, QFont, QGraphicsScene, QGraphicsView, QIcon, QPainterPath, QPen, QPixmap, QPushButton

class ImageView(TopicMessageView):
    """
    Popup image viewer
    """
    name = 'Image'
    def __init__(self, timeline, parent):
        super(ImageView, self).__init__(timeline, parent)

        self._image_lock  = threading.RLock()
        self._image = None
        self._image_topic = None
        self._image_stamp = None
        self.quality = Image.NEAREST  # quality hint for scaling

        # TODO put the image_topic and image_stamp on the picture or display them in some fashion
        self._overlay_font_size = 14.0
        self._overlay_indent = (18, 18)
        self._overlay_brush = QBrush(Qt.blue, Qt.SolidPattern)
        self._overlay_pen = QPen(Qt.blue)

        self._image_view = QGraphicsView(parent)
        self._image_view.resizeEvent = self._resizeEvent
        self._scene = QGraphicsScene()
        self._image_view.setScene(self._scene)
        parent.layout().addWidget(self._image_view)
        
        self._save_button = QPushButton(QIcon.fromTheme('document-save'),'', self.toolbar)
        self._save_button.setToolTip('Save current image/Save images from selected region')
        self._save_button.clicked[bool].connect(self._handle_save_clicked)
        self.toolbar.addWidget(self._save_button)

        self._resize_button = QPushButton(QIcon.fromTheme('view-restore'),'', self.toolbar)
        self._resize_button.setToolTip('Restore to Original size')
        self._resize_button.clicked[bool].connect(self._handle_restore_clicked)
        self.toolbar.addWidget(self._resize_button)

        self._next_frame_num = 1
        self._filename = None

    def _handle_save_clicked(self, checked):
        if self._filename:
            self.save_frame(self._filename)
        else:
            filename = QFileDialog.getSaveFileName(self.parent, self.tr('Save images with prefix...'), '.', self.tr('PNG files {*.png} (*.png)'))
            if filename[0] != '':
                self._filename = filename[0]
                self.save_frame(self._filename)

    def _handle_restore_clicked(self, checked):
        with self._image_lock:
            if self._image:
                self.parent.resize(self._image.size[0], self._image.size[1])

    def save_frame(self, name):
        if not self._image:
            return

        file_spec = name
        if file_spec[-4:] == '.png':
            file_spec = file_spec[:-4]
        filename = file_spec + '_' + str(self._next_frame_num).zfill(3) + '.png'

        with self._image_lock:
            if self._image_msg:
                self._save_image_msg(self._image_msg, filename)
                self._next_frame_num += 1

    def _save_image_msg(self, img_msg, filename):
        pil_img = image_helper.imgmsg_to_pil(img_msg, rgba=False)
        if pil_img.mode in ('RGB', 'RGBA'):
            pil_img = pil_img.convert('RGB')
            pil_img = image_helper.pil_bgr2rgb(pil_img)
            pil_img = pil_img.convert('RGB')
        pil_img.save(filename)

    # MessageView implementation
    def _resizeEvent(self, event):
        self._scene.setSceneRect(0, 0, self._image_view.size().width() - 2, self._image_view.size().height() - 2)
        self.put_image_into_scene()

    def message_viewed(self, bag, msg_details):
        """
        refreshes the image
        """
        TopicMessageView.message_viewed(self, bag, msg_details)
        topic, msg, t = msg_details[:3]
        if not msg:
            self.set_image(None, topic, '')
        else:
            self.set_image(msg, topic, msg.header.stamp)

    def message_cleared(self):
        TopicMessageView.message_cleared(self)
        self.set_image(None, None, None)

    # End MessageView implementation
    def put_image_into_scene(self):
        with self._image_lock:
            if self._image:
                resized_image = self._image.resize((self._image_view.size().width()-2, self._image_view.size().height()-2), self.quality)

                QtImage = ImageQt.ImageQt(resized_image)
                pixmap = QPixmap.fromImage(QtImage)
                self._scene.clear()
                self._scene.addPixmap(pixmap)

                path = QPainterPath()
                path.addText(self._overlay_indent[0], self._overlay_indent[1], QFont("cairo"), bag_helper.stamp_to_str(self._image_stamp))
                self._scene.addPath(path, self._overlay_pen, self._overlay_brush)

    def set_image(self, image_msg, image_topic, image_stamp):
        with self._image_lock:
            self._image_msg = image_msg
            if image_msg:
                self._image = image_helper.imgmsg_to_pil(image_msg)
            else:
                self._image = None
            self._image_topic = image_topic
            self._image_stamp = image_stamp
            self.put_image_into_scene()
