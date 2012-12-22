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

import rospy

from python_qt_binding.QtGui import QIcon, QImage, QMessageBox, QPainter, QPixmap
from python_qt_binding.QtSvg import QSvgRenderer


def dashinfo(msg, obj, title='Info'):
    """
    Logs a message with ``rospy.loginfo`` and displays a ``QMessageBox`` to the user

    :param msg: Message to display.
    :type msg: str
    :param obj: Parent object for the ``QMessageBox``
    :type obj: QObject
    :param title: An optional title for the `QMessageBox``
    :type title: str
    """
    rospy.loginfo(msg)

    box = QMessageBox()
    box.setText(msg)
    box.setWindowTitle(title)
    box.show()

    obj._message_box = box


def dashwarn(msg, obj, title='Warning'):
    """
    Logs a message with ``rospy.logwarn`` and displays a ``QMessageBox`` to the user

    :param msg: Message to display.
    :type msg: str
    :param obj: Parent object for the ``QMessageBox``
    :type obj: QObject
    :param title: An optional title for the `QMessageBox``
    :type title: str
    """
    rospy.logwarn(msg)

    box = QMessageBox()
    box.setText(msg)
    box.setWindowTitle(title)
    box.show()

    obj._message_box = box


def dasherr(msg, obj, title='Error'):
    """
    Logs a message with ``rospy.logerr`` and displays a ``QMessageBox`` to the user

    :param msg: Message to display.
    :type msg: str
    :param obj: Parent object for the ``QMessageBox``
    :type obj: QObject
    :param title: An optional title for the `QMessageBox``
    :type title: str
    """
    rospy.logerr(msg)

    box = QMessageBox()
    box.setText(msg)
    box.setWindowTitle(title)
    box.show()

    obj._message_box = box


class IconHelper(object):
    """
    Helper class to easily access images and build QIcons out of lists of file names
    """
    def __init__(self, paths=None):
        self._image_paths = paths if paths else []

    def add_image_path(self, path):
        """
        Paths added will be searched for images by the _find_image function
        Paths will be searched in revearse order by add time
        The last path to be searched is always rqt_robot_dashboard/images
        Subdirectories are not recursively searched

        :param path: The path to add to the image paths list
        :type path: str
        """
        self._image_paths = [path] + self._image_paths

    def make_icon(self, image_list, mode=QIcon.Normal, state=QIcon.On):
        """
        Helper function to create QIcons from lists of image files
        Warning: svg files interleaved with other files will not render correctly

        :param image_list: list of image paths to layer into an icon.
        :type image: list of str
        :param mode: The mode of the QIcon to be created.
        :type mode: int
        :param state: the state of the QIcon to be created.
        :type state: int
        """
        if type(image_list) is not list:
            image_list = [image_list]
        if len(image_list) <= 0:
            raise TypeError('The list of images is empty.')

        num_svg = 0
        for item in image_list:
            if item[-4:].lower() == '.svg':
                num_svg = num_svg + 1

        if num_svg != len(image_list):
            # Legacy support for non-svg images
            icon_pixmap = QPixmap()
            icon_pixmap.load(image_list[0])
            painter = QPainter(icon_pixmap)
            for item in image_list[1:]:
                painter.drawPixmap(0, 0, QPixmap(item))
            icon = QIcon()
            icon.addPixmap(icon_pixmap, mode, state)
            painter.end()
            return icon
        else:
            #  rendering SVG files into a QImage
            renderer = QSvgRenderer(image_list[0])
            icon_image = QImage(renderer.defaultSize(), QImage.Format_ARGB32)
            icon_image.fill(0)
            painter = QPainter(icon_image)
            renderer.render(painter)
            if len(image_list) > 1:
                for item in image_list[1:]:
                    renderer.load(item)
                    renderer.render(painter)
            painter.end()
            #  Convert QImage into a pixmap to create the icon
            icon_pixmap = QPixmap()
            icon_pixmap.convertFromImage(icon_image)
            icon = QIcon(icon_pixmap)
            return icon

    def find_image(self, path):
        """
        Convenience function to help with finding images.
        Path can either be specified as absolute paths or relative to any path in ``_image_paths``

        :param path: The path or name of the image.
        :type path: str
        """
        if os.path.exists(path):
            return path
        for image_path in self._image_paths:
            if os.path.exists(os.path.join(image_path, path)):
                return os.path.join(image_path, path)
            elif '.' in path and os.path.exists(os.path.join(image_path, 'nonsvg', path)):
                return os.path.join(image_path, 'nonsvg', path)
        return os.path.join(self._image_paths[-1], 'ic-missing-icon.svg')

    def build_icon(self, image_name_list, mode=QIcon.Normal, state=QIcon.On):
        """
        Convenience function to create an icon from a list of file names

        :param image_name_list: List of file image names to make into an icon
        :type image_name_list: list of str
        :param mode: The mode of the QIcon to be created.
        :type mode: int
        :param state: the state of the QIcon to be created.
        :type state: int
        """
        found_list = []
        for name in image_name_list:
            found_list.append(self.find_image(name))
        return self.make_icon(found_list, mode, state)
