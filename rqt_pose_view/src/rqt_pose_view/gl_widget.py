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
import math
import numpy

from python_qt_binding.QtCore import QPoint, Qt
from python_qt_binding.QtOpenGL import QGLFormat, QGLWidget

import OpenGL
OpenGL.ERROR_CHECKING = True
from OpenGL.GL import glClear, glClearColor, glEnable, glGetDoublev, glLoadIdentity, glLoadMatrixd, glMatrixMode, glMultMatrixd, glRotated, glTranslated, glTranslatef, glViewport, GL_COLOR_BUFFER_BIT, GL_DEPTH_BUFFER_BIT, GL_DEPTH_TEST, GL_MODELVIEW, GL_MODELVIEW_MATRIX, GL_PROJECTION
from OpenGL.GLU import gluPerspective


class GLWidget(QGLWidget):

    def __init__(self, parent=None):
        glformat = QGLFormat()
        glformat.setSampleBuffers(True)
        super(GLWidget, self).__init__(glformat, parent)

        self.setCursor(Qt.OpenHandCursor)
        self.setMouseTracking(True)

        self._modelview_matrix = numpy.identity(4)
        self._near = 0.1
        self._far = 100.0
        self._fovy = 45.0
        self._radius = 5.0
        self._last_point_2d = QPoint()
        self._last_point_3d = [0.0, 0.0, 0.0]
        self._last_point_3d_ok = False

    def initializeGL(self):
        glClearColor(0.0, 0.0, 0.0, 0.0)
        glEnable(GL_DEPTH_TEST)

    def resizeGL(self, width, height):
        glViewport(0, 0, width, height)
        self.set_projection(self._near, self._far, self._fovy)
        self.updateGL()

    def paintGL(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glMatrixMode(GL_MODELVIEW)
        glLoadMatrixd(self._modelview_matrix)

    def get_view_matrix(self):
        return self._modelview_matrix.tolist()

    def set_view_matrix(self, matrix):
        self._modelview_matrix = numpy.array(matrix)

    def set_projection(self, near, far, fovy):
        self._near = near
        self._far = far
        self._fovy = fovy
        self.makeCurrent()
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        height = max(self.height(), 1)
        gluPerspective(self._fovy, float(self.width()) / float(height), self._near, self._far)
        self.updateGL()

    def reset_view(self):
        # scene pos and size
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        self._modelview_matrix = glGetDoublev(GL_MODELVIEW_MATRIX)
        self.view_all()

    def reset_rotation(self):
        self._modelview_matrix[0] = [1.0, 0.0, 0.0, 0.0]
        self._modelview_matrix[1] = [0.0, 1.0, 0.0, 0.0]
        self._modelview_matrix[2] = [0.0, 0.0, 1.0, 0.0]
        glMatrixMode(GL_MODELVIEW)
        glLoadMatrixd(self._modelview_matrix)

    def translate(self, trans):
        # translate the object
        self.makeCurrent()
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        glTranslated(trans[0], trans[1], trans[2])
        glMultMatrixd(self._modelview_matrix)
        # update _modelview_matrix
        self._modelview_matrix = glGetDoublev(GL_MODELVIEW_MATRIX)

    def rotate(self, axis, angle):
        # rotate the object
        self.makeCurrent()
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        t = [self._modelview_matrix[3][0], self._modelview_matrix[3][1], self._modelview_matrix[3][2]]
        glTranslatef(t[0], t[1], t[2])
        glRotated(angle, axis[0], axis[1], axis[2])
        glTranslatef(-t[0], -t[1], -t[2])
        glMultMatrixd(self._modelview_matrix)
        # update _modelview_matrix
        self._modelview_matrix = glGetDoublev(GL_MODELVIEW_MATRIX)

    def view_all(self):
        self.translate([-self._modelview_matrix[0][3], -self._modelview_matrix[1][3], -self._modelview_matrix[2][3] - self._radius / 2.0])

    def wheelEvent(self, event):
        # only zoom when no mouse buttons are pressed, to prevent interference with other user interactions
        if event.buttons() == Qt.NoButton:
            d = float(event.delta()) / 200.0 * self._radius
            self.translate([0.0, 0.0, d])
            self.updateGL()
            event.accept()

    def mousePressEvent(self, event):
        self._last_point_2d = event.pos()
        self._last_point_3d_ok, self._last_point_3d = self._map_to_sphere(self._last_point_2d)

    def mouseMoveEvent(self, event):
        new_point_2d = event.pos()

        if not self.rect().contains(new_point_2d):
            return

        new_point_3d_ok, new_point_3d = self._map_to_sphere(new_point_2d)

        dy = float(new_point_2d.y() - self._last_point_2d.y())
        h = float(self.height())

        # left button: rotate around center
        if event.buttons() == Qt.LeftButton and event.modifiers() == Qt.NoModifier:
            if self._last_point_3d_ok and new_point_3d_ok:
                cos_angle = numpy.dot(self._last_point_3d, new_point_3d)
                if abs(cos_angle) < 1.0:
                    axis = numpy.cross(self._last_point_3d, new_point_3d)
                    angle = 2.0 * math.acos(cos_angle) * 180.0 / math.pi
                    self.rotate(axis, angle)

        # middle button (or left + shift): move in x-y-direction
        elif event.buttons() == Qt.MidButton or (event.buttons() == Qt.LeftButton and event.modifiers() == Qt.ShiftModifier):
            dx = float(new_point_2d.x() - self._last_point_2d.x())
            w = float(self.width())
            z = -self._modelview_matrix[3][2] / self._modelview_matrix[3][3]
            n = 0.01 * self._radius
            up = math.tan(self._fovy / 2.0 * math.pi / 180.0) * n
            right = up * w / h
            self.translate([2.0 * dx / w * right / n * z, -2.0 * dy / h * up / n * z, 0.0])

        # left and middle button (or left + ctrl): move in z-direction
        elif event.buttons() == (Qt.LeftButton | Qt.MidButton) or (event.buttons() == Qt.LeftButton and event.modifiers() == Qt.ControlModifier):
            delta_z = self._radius * dy * 2.0 / h
            self.translate([0.0, 0.0, delta_z])

        # remember the new points and flag
        self._last_point_2d = new_point_2d
        self._last_point_3d = new_point_3d
        self._last_point_3d_ok = new_point_3d_ok

        # trigger redraw
        self.updateGL()

    def mouseReleaseEvent(self, _event):
        self._last_point_3d_ok = False

    def _map_to_sphere(self, pos):
        v = [0.0, 0.0, 0.0]
        # check if inside widget
        if self.rect().contains(pos):
            # map widget coordinates to the centered unit square [-0.5..0.5] x [-0.5..0.5]
            v[0] = float(pos.x() - 0.5 * self.width()) / self.width()
            v[1] = float(0.5 * self.height() - pos.y()) / self.height()
            # use Pythagoras to compute z (the sphere has radius sqrt(2.0*0.5*0.5))
            v[2] = math.sqrt(max(0.5 - v[0] * v[0] - v[1] * v[1], 0.0))
            # normalize direction to unit sphere
            v = numpy.array(v) / numpy.linalg.norm(v)
            return True, v
        else:
            return False, v
