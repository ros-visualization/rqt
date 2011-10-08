import os

from rosgui.QtBindingHelper import loadUi
from QtCore import QTimer, Qt, Slot
from QtGui import QDockWidget, QHBoxLayout
from QtOpenGL import QGLWidget, QGLFormat, QGL

import roslib
roslib.load_manifest('rosgui_pose_view')
import rospy
from rostopic import get_topic_class

from math3D import toMatrixQ
from OpenGL.GL import *
import PyGLWidget
reload(PyGLWidget) # force reload to update on changes during runtime

# main class inherits from the ui window class
class PoseViewWidget(QDockWidget):

    def __init__(self, plugin, plugin_context):
        QDockWidget.__init__(self, plugin_context.main_window())
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'PoseViewWidget.ui')
        loadUi(ui_file, self)
        self.plugin_ = plugin

        self.orientation_ = [1.0, 0.0, 0.0, 0.0]
        self.topic_name_ = None
        self.subscriber_ = None

        # create GL view         
        self.gl_view_ = PyGLWidget.PyGLWidget()
        self.gl_view_.setAcceptDrops(True)

        # backup and replace original paint method
        self.gl_view_.paintGL_ = self.gl_view_.paintGL
        self.gl_view_.paintGL = self.paintGL

        # add GL view to widget layout        
        self.dockWidgetContents.layout().addWidget(self.gl_view_)

        # init and start update timer at 40Hz (25fps)
        self.timerUpdate = QTimer(self)
        self.timerUpdate.timeout.connect(self.update_timeout)
        self.timerUpdate.start(40)


    def save_settings(self, global_settings, perspective_settings):
        view_matrix_string = repr(self.gl_view_.get_view_matrix())
        perspective_settings.set_value('view_matrix', view_matrix_string)


    def restore_settings(self, global_settings, perspective_settings):
        view_matrix_string = perspective_settings.value('view_matrix')
        try:
            # TODO: re-enable only after resetting camera is possible
            raise Exception
            view_matrix = eval(view_matrix_string)
        except:
            view_matrix = None

        if view_matrix is None:
            self.gl_view_.makeCurrent()
            self.gl_view_.reset_view()
            self.gl_view_.reset_rotation()
            self.gl_view_.rotate((1, 0, 0), 270)
            self.gl_view_.translate((-1, -4, -10))
        else:
            self.gl_view_.set_view_matrix(view_matrix)


    def message_callback(self, message):
        self.position_ = [message.position.x, message.position.y, message.position.z]
        self.orientation_ = [message.orientation.x, message.orientation.y, message.orientation.z, message.orientation.w]
        #print 'received:', self.orientation_


    def update_timeout(self):
        self.gl_view_.makeCurrent()
        self.gl_view_.updateGL()


    def paintGL(self):
        self.gl_view_.paintGL_()
        self.paintGLGrid()
        self.paintGLCoorsystem()
        self.paintGLBox()


    def paintGLBox(self):
        #glTranslatef(*self.position_)     # Move Box
        glTranslatef(2.0, 2.0, 2.0)       # Move Box
        matrix = toMatrixQ(self.orientation_) # convert quaternion to rotation matrix
        glMultMatrixf(matrix)             # Rotate Box 

        glBegin(GL_QUADS)                 # Start Drawing The Box

        glColor3f(0.0, 1.0, 0.0)
        glVertex3f( 1.0, 1.0,-1.0)        # Top Right Of The Quad (Top)
        glVertex3f(-1.0, 1.0,-1.0)        # Top Left Of The Quad (Top)
        glVertex3f(-1.0, 1.0, 1.0)        # Bottom Left Of The Quad (Top)
        glVertex3f( 1.0, 1.0, 1.0)        # Bottom Right Of The Quad (Top)

        glColor3f(0.5, 1.0, 0.5)
        glVertex3f( 1.0,-1.0, 1.0)        # Top Right Of The Quad (Bottom)
        glVertex3f(-1.0,-1.0, 1.0)        # Top Left Of The Quad (Bottom)
        glVertex3f(-1.0,-1.0,-1.0)        # Bottom Left Of The Quad (Bottom)
        glVertex3f( 1.0,-1.0,-1.0)        # Bottom Right Of The Quad (Bottom)

        glColor3f(0.0, 0.0, 1.0)
        glVertex3f( 1.0, 1.0, 1.0)        # Top Right Of The Quad (Front)
        glVertex3f(-1.0, 1.0, 1.0)        # Top Left Of The Quad (Front)
        glVertex3f(-1.0,-1.0, 1.0)        # Bottom Left Of The Quad (Front)
        glVertex3f( 1.0,-1.0, 1.0)        # Bottom Right Of The Quad (Front)

        glColor3f(0.5, 0.5, 1.0)
        glVertex3f( 1.0,-1.0,-1.0)        # Bottom Left Of The Quad (Back)
        glVertex3f(-1.0,-1.0,-1.0)        # Bottom Right Of The Quad (Back)
        glVertex3f(-1.0, 1.0,-1.0)        # Top Right Of The Quad (Back)
        glVertex3f( 1.0, 1.0,-1.0)        # Top Left Of The Quad (Back)

        glColor3f(1.0, 0.5, 0.5)
        glVertex3f(-1.0, 1.0, 1.0)        # Top Right Of The Quad (Left)
        glVertex3f(-1.0, 1.0,-1.0)        # Top Left Of The Quad (Left)
        glVertex3f(-1.0,-1.0,-1.0)        # Bottom Left Of The Quad (Left)
        glVertex3f(-1.0,-1.0, 1.0)        # Bottom Right Of The Quad (Left)

        glColor3f(1.0, 0.0, 0.0)
        glVertex3f( 1.0, 1.0,-1.0)        # Top Right Of The Quad (Right)
        glVertex3f( 1.0, 1.0, 1.0)        # Top Left Of The Quad (Right)
        glVertex3f( 1.0,-1.0, 1.0)        # Bottom Left Of The Quad (Right)
        glVertex3f( 1.0,-1.0,-1.0)        # Bottom Right Of The Quad (Right)
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
            if ros_topic_name == None:
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
        if self.subscriber_:
            self.subscriber_.unregister()


    def subscribe_topic(self, topic_name):
        print 'subscribing:', topic_name
        msg_class, self.topic_name_, _ = get_topic_class(topic_name)
        self.subscriber_ = rospy.Subscriber(self.topic_name_, msg_class, self.message_callback)


    # override Qt's closeEvent() method to trigger plugin unloading
    def closeEvent(self, event):
        self.unregister_topic()
        QDockWidget.closeEvent(self, event)
        if event.isAccepted():
            self.plugin_.deleteLater()
