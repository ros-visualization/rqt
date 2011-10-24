from __future__ import division
import os

from rosgui.QtBindingHelper import loadUi
from QtCore import qDebug, Qt, QTimer, Slot
from QtGui import QDockWidget

import roslib
roslib.load_manifest('rosgui_pose_view')
import rospy
from rostopic import get_topic_class

from math3D import toMatrixQ
from OpenGL.GL import glBegin, glColor3f, glEnd, glLineWidth, glMultMatrixf, glTranslatef, glVertex3f, GL_LINES, GL_QUADS
import PyGLWidget
reload(PyGLWidget) # force reload to update on changes during runtime

# main class inherits from the ui window class
class PoseViewWidget(QDockWidget):

    def __init__(self, plugin, plugin_context):
        super(PoseViewWidget, self).__init__(plugin_context.main_window())
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'PoseViewWidget.ui')
        loadUi(ui_file, self)
        self._plugin = plugin

        self._position = [0.0, 0.0, 0.0]
        self._orientation = [1.0, 0.0, 0.0, 0.0]
        self._topic_name = None
        self._subscriber = None

        # create GL view         
        self._gl_view = PyGLWidget.PyGLWidget()
        self._gl_view.setAcceptDrops(True)

        # backup and replace original paint method
        self._gl_view.paintGL_original = self._gl_view.paintGL
        self._gl_view.paintGL = self.paintGL

        # add GL view to widget layout        
        self.dockWidgetContents.layout().addWidget(self._gl_view)

        # init and start update timer at 40Hz (25fps)
        self._update_timer = QTimer(self)
        self._update_timer.timeout.connect(self.update_timeout)
        self._update_timer.start(40)


    def save_settings(self, global_settings, perspective_settings):
        view_matrix_string = repr(self._gl_view.get_view_matrix())
        perspective_settings.set_value('view_matrix', view_matrix_string)


    def restore_settings(self, global_settings, perspective_settings):
        #view_matrix_string = perspective_settings.value('view_matrix')
        try:
            # TODO: re-enable only after resetting camera is possible
            #view_matrix = eval(view_matrix_string)
            raise Exception
        except Exception:
            view_matrix = None

        if view_matrix is None:
            self._gl_view.makeCurrent()
            self._gl_view.reset_view()
            self._gl_view.reset_rotation()
            self._gl_view.rotate((1, 0, 0), 270)
            self._gl_view.translate((-1, -4, -10))
        else:
            self._gl_view.set_view_matrix(view_matrix)


    def message_callback(self, message):
        self._position = [message.position.x, message.position.y, message.position.z]
        self._orientation = [message.orientation.x, message.orientation.y, message.orientation.z, message.orientation.w]
        #print 'received:', self._orientation


    def update_timeout(self):
        self._gl_view.makeCurrent()
        self._gl_view.updateGL()


    def paintGL(self):
        self._gl_view.paintGL_original()
        self.paintGLGrid()
        self.paintGLCoorsystem()
        self.paintGLBox()


    def paintGLBox(self):
        #glTranslatef(*self._position)     # Move Box
        glTranslatef(2.0, 2.0, 2.0)       # Move Box
        matrix = toMatrixQ(self._orientation) # convert quaternion to rotation matrix
        glMultMatrixf(matrix)             # Rotate Box 

        glBegin(GL_QUADS)                 # Start Drawing The Box

        glColor3f(0.0, 1.0, 0.0)
        glVertex3f(1.0, 1.0, -1.0)        # Top Right Of The Quad (Top)
        glVertex3f(-1.0, 1.0, -1.0)        # Top Left Of The Quad (Top)
        glVertex3f(-1.0, 1.0, 1.0)        # Bottom Left Of The Quad (Top)
        glVertex3f(1.0, 1.0, 1.0)        # Bottom Right Of The Quad (Top)

        glColor3f(0.5, 1.0, 0.5)
        glVertex3f(1.0, -1.0, 1.0)        # Top Right Of The Quad (Bottom)
        glVertex3f(-1.0, -1.0, 1.0)        # Top Left Of The Quad (Bottom)
        glVertex3f(-1.0, -1.0, -1.0)        # Bottom Left Of The Quad (Bottom)
        glVertex3f(1.0, -1.0, -1.0)        # Bottom Right Of The Quad (Bottom)

        glColor3f(0.0, 0.0, 1.0)
        glVertex3f(1.0, 1.0, 1.0)        # Top Right Of The Quad (Front)
        glVertex3f(-1.0, 1.0, 1.0)        # Top Left Of The Quad (Front)
        glVertex3f(-1.0, -1.0, 1.0)        # Bottom Left Of The Quad (Front)
        glVertex3f(1.0, -1.0, 1.0)        # Bottom Right Of The Quad (Front)

        glColor3f(0.5, 0.5, 1.0)
        glVertex3f(1.0, -1.0, -1.0)        # Bottom Left Of The Quad (Back)
        glVertex3f(-1.0, -1.0, -1.0)        # Bottom Right Of The Quad (Back)
        glVertex3f(-1.0, 1.0, -1.0)        # Top Right Of The Quad (Back)
        glVertex3f(1.0, 1.0, -1.0)        # Top Left Of The Quad (Back)

        glColor3f(1.0, 0.5, 0.5)
        glVertex3f(-1.0, 1.0, 1.0)        # Top Right Of The Quad (Left)
        glVertex3f(-1.0, 1.0, -1.0)        # Top Left Of The Quad (Left)
        glVertex3f(-1.0, -1.0, -1.0)        # Bottom Left Of The Quad (Left)
        glVertex3f(-1.0, -1.0, 1.0)        # Bottom Right Of The Quad (Left)

        glColor3f(1.0, 0.0, 0.0)
        glVertex3f(1.0, 1.0, -1.0)        # Top Right Of The Quad (Right)
        glVertex3f(1.0, 1.0, 1.0)        # Top Left Of The Quad (Right)
        glVertex3f(1.0, -1.0, 1.0)        # Bottom Left Of The Quad (Right)
        glVertex3f(1.0, -1.0, -1.0)        # Bottom Right Of The Quad (Right)
        glEnd()                           # Done Drawing The Quad


    def paintGLGrid(self):
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


    def paintGLCoorsystem(self):
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


    @Slot('QDragEnterEvent*')
    def dragEnterEvent(self, event):
        if not event.mimeData().hasText():
            if not hasattr(event.source(), 'selectedItems') or len(event.source().selectedItems()) == 0:
                qDebug('Plot.dragEnterEvent(): not hasattr(event.source(), selectedItems) or len(event.source().selectedItems()) == 0')
                return
            item = event.source().selectedItems()[0]
            ros_topic_name = item.data(0, Qt.UserRole)
            if ros_topic_name is None:
                qDebug('Plot.dragEnterEvent(): not hasattr(item, ros_topic_name_)')
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
        print 'subscribing:', topic_name
        msg_class, self._topic_name, _ = get_topic_class(topic_name)
        self._subscriber = rospy.Subscriber(self._topic_name, msg_class, self.message_callback)


    # override Qt's closeEvent() method to trigger _plugin unloading
    def closeEvent(self, event):
        self.unregister_topic()
        event.ignore()
        self._plugin.deleteLater()
