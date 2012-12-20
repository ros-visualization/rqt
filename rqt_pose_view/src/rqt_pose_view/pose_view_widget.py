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

from __future__ import division
import os
import rospkg

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, qWarning, Slot
from python_qt_binding.QtGui import QAction, QMenu, QWidget

import rospy
from rostopic import get_topic_class
from tf.transformations import quaternion_matrix, quaternion_about_axis

from OpenGL.GL import glBegin, glColor3f, glEnd, glLineWidth, glMultMatrixf, glTranslatef, glVertex3f, GL_LINES, GL_QUADS
from .gl_widget import GLWidget


# main class inherits from the ui window class
class PoseViewWidget(QWidget):

    def __init__(self, plugin):
        super(PoseViewWidget, self).__init__()
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_pose_view'), 'resource', 'PoseViewWidget.ui')
        loadUi(ui_file, self)
        self._plugin = plugin

        self._position = (0.0, 0.0, 0.0)
        self._orientation = quaternion_about_axis(0.0, (1.0, 0.0, 0.0))
        self._topic_name = None
        self._subscriber = None

        # create GL view
        self._gl_view = GLWidget()
        self._gl_view.setAcceptDrops(True)

        # backup and replace original paint method
        self._gl_view.paintGL_original = self._gl_view.paintGL
        self._gl_view.paintGL = self._gl_view_paintGL

        # backup and replace original mouse release method
        self._gl_view.mouseReleaseEvent_original = self._gl_view.mouseReleaseEvent
        self._gl_view.mouseReleaseEvent = self._gl_view_mouseReleaseEvent

        # add GL view to widget layout
        self.layout().addWidget(self._gl_view)

        # init and start update timer with 40ms (25fps)
        self._update_timer = QTimer(self)
        self._update_timer.timeout.connect(self.update_timeout)
        self._update_timer.start(40)

    def save_settings(self, plugin_settings, instance_settings):
        view_matrix_string = repr(self._gl_view.get_view_matrix())
        instance_settings.set_value('view_matrix', view_matrix_string)

    def restore_settings(self, plugin_settings, instance_settings):
        view_matrix_string = instance_settings.value('view_matrix')
        try:
            view_matrix = eval(view_matrix_string)
        except Exception:
            view_matrix = None

        if view_matrix is not None:
            self._gl_view.set_view_matrix(view_matrix)
        else:
            self._set_default_view()

    def _set_default_view(self):
        self._gl_view.makeCurrent()
        self._gl_view.reset_view()
        self._gl_view.rotate((0, 0, 1), 45)
        self._gl_view.rotate((1, 0, 0), -45)
        self._gl_view.translate((0, -3, -15))

    def message_callback(self, message):
        self._position = (message.position.x, message.position.y, message.position.z)
        self._orientation = (message.orientation.x, message.orientation.y, message.orientation.z, message.orientation.w)

    def update_timeout(self):
        self._gl_view.makeCurrent()
        self._gl_view.updateGL()

    def _gl_view_paintGL(self):
        self._gl_view.paintGL_original()
        self._paintGLGrid()
        self._paintGLCoorsystem()
        self._paintGLBox()

    def _paintGLBox(self):
        self._position = (2.0, 2.0, 2.0)  # Set fixed translation for now
        glTranslatef(*self._position)     # Translate Box

        matrix = quaternion_matrix(self._orientation)  # convert quaternion to translation matrix
        glMultMatrixf(matrix)             # Rotate Box

        glBegin(GL_QUADS)                 # Start Drawing The Box

        glColor3f(0.0, 1.0, 0.0)
        glVertex3f(1.0, 1.0, -1.0)        # Top Right Of The Quad (Top)
        glVertex3f(-1.0, 1.0, -1.0)       # Top Left Of The Quad (Top)
        glVertex3f(-1.0, 1.0, 1.0)        # Bottom Left Of The Quad (Top)
        glVertex3f(1.0, 1.0, 1.0)         # Bottom Right Of The Quad (Top)

        glColor3f(0.5, 1.0, 0.5)
        glVertex3f(1.0, -1.0, 1.0)        # Top Right Of The Quad (Bottom)
        glVertex3f(-1.0, -1.0, 1.0)       # Top Left Of The Quad (Bottom)
        glVertex3f(-1.0, -1.0, -1.0)      # Bottom Left Of The Quad (Bottom)
        glVertex3f(1.0, -1.0, -1.0)       # Bottom Right Of The Quad (Bottom)

        glColor3f(0.0, 0.0, 1.0)
        glVertex3f(1.0, 1.0, 1.0)         # Top Right Of The Quad (Front)
        glVertex3f(-1.0, 1.0, 1.0)        # Top Left Of The Quad (Front)
        glVertex3f(-1.0, -1.0, 1.0)       # Bottom Left Of The Quad (Front)
        glVertex3f(1.0, -1.0, 1.0)        # Bottom Right Of The Quad (Front)

        glColor3f(0.5, 0.5, 1.0)
        glVertex3f(1.0, -1.0, -1.0)       # Bottom Left Of The Quad (Back)
        glVertex3f(-1.0, -1.0, -1.0)      # Bottom Right Of The Quad (Back)
        glVertex3f(-1.0, 1.0, -1.0)       # Top Right Of The Quad (Back)
        glVertex3f(1.0, 1.0, -1.0)        # Top Left Of The Quad (Back)

        glColor3f(1.0, 0.5, 0.5)
        glVertex3f(-1.0, 1.0, 1.0)        # Top Right Of The Quad (Left)
        glVertex3f(-1.0, 1.0, -1.0)       # Top Left Of The Quad (Left)
        glVertex3f(-1.0, -1.0, -1.0)      # Bottom Left Of The Quad (Left)
        glVertex3f(-1.0, -1.0, 1.0)       # Bottom Right Of The Quad (Left)

        glColor3f(1.0, 0.0, 0.0)
        glVertex3f(1.0, 1.0, -1.0)        # Top Right Of The Quad (Right)
        glVertex3f(1.0, 1.0, 1.0)         # Top Left Of The Quad (Right)
        glVertex3f(1.0, -1.0, 1.0)        # Bottom Left Of The Quad (Right)
        glVertex3f(1.0, -1.0, -1.0)       # Bottom Right Of The Quad (Right)
        glEnd()                           # Done Drawing The Quad

    def _paintGLGrid(self):
        resolutionMillimeters = 1
        griddedAreaSize = 100

        glLineWidth(1.0)

        glBegin(GL_LINES)

        glColor3f(1.0, 1.0, 1.0)

        glVertex3f(griddedAreaSize, 0, 0)
        glVertex3f(-griddedAreaSize, 0, 0)
        glVertex3f(0, griddedAreaSize, 0)
        glVertex3f(0, -griddedAreaSize, 0)

        numOfLines = int(griddedAreaSize / resolutionMillimeters)

        for i in range(numOfLines):
            glVertex3f(resolutionMillimeters * i, -griddedAreaSize, 0)
            glVertex3f(resolutionMillimeters * i, griddedAreaSize, 0)
            glVertex3f(griddedAreaSize, resolutionMillimeters * i, 0)
            glVertex3f(-griddedAreaSize, resolutionMillimeters * i, 0)

            glVertex3f(resolutionMillimeters * (-i), -griddedAreaSize, 0)
            glVertex3f(resolutionMillimeters * (-i), griddedAreaSize, 0)
            glVertex3f(griddedAreaSize, resolutionMillimeters * (-i), 0)
            glVertex3f(-griddedAreaSize, resolutionMillimeters * (-i), 0)

        glEnd()

    def _paintGLCoorsystem(self):
        glLineWidth(10.0)

        glBegin(GL_LINES)

        glColor3f(1.0, 0.0, 0.0)
        glVertex3f(0.0, 0.0, 0.0)
        glVertex3f(1.0, 0.0, 0.0)

        glColor3f(0.0, 1.0, 0.0)
        glVertex3f(0.0, 0.0, 0.0)
        glVertex3f(0.0, 1.0, 0.0)

        glColor3f(0.0, 0.0, 1.0)
        glVertex3f(0.0, 0.0, 0.0)
        glVertex3f(0.0, 0.0, 1.0)

        glEnd()

    def _gl_view_mouseReleaseEvent(self, event):
        if event.button() == Qt.RightButton:
            menu = QMenu(self._gl_view)
            action = QAction(self._gl_view.tr("Reset view"), self._gl_view)
            menu.addAction(action)
            action.triggered.connect(self._set_default_view)
            menu.exec_(self._gl_view.mapToGlobal(event.pos()))

    @Slot('QDragEnterEvent*')
    def dragEnterEvent(self, event):
        if not event.mimeData().hasText():
            if not hasattr(event.source(), 'selectedItems') or len(event.source().selectedItems()) == 0:
                qWarning('Plot.dragEnterEvent(): not hasattr(event.source(), selectedItems) or len(event.source().selectedItems()) == 0')
                return
            item = event.source().selectedItems()[0]
            ros_topic_name = item.data(0, Qt.UserRole)
            if ros_topic_name is None:
                qWarning('Plot.dragEnterEvent(): not hasattr(item, ros_topic_name_)')
                return
        # TODO: do some checks for valid topic here
        event.acceptProposedAction()

    @Slot('QDropEvent*')
    def dropEvent(self, event):
        if event.mimeData().hasText():
            topic_name = str(event.mimeData().text())
        else:
            droped_item = event.source().selectedItems()[0]
            topic_name = str(droped_item.data(0, Qt.UserRole))

        self.unregister_topic()
        self.subscribe_topic(topic_name)

    def unregister_topic(self):
        if self._subscriber:
            self._subscriber.unregister()

    def subscribe_topic(self, topic_name):
        msg_class, self._topic_name, _ = get_topic_class(topic_name)
        self._subscriber = rospy.Subscriber(self._topic_name, msg_class, self.message_callback)

    def shutdown_plugin(self):
        self.unregister_topic()
