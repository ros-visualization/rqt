from __future__ import division
import os

from rosgui.QtBindingHelper import loadUi
from QtCore import QEvent, QObject, Qt, QTimer, Slot
from QtGui import QDockWidget, QShortcut

import roslib
roslib.load_manifest('rosgui_robot_steering')
import rospy

from geometry_msgs.msg import Twist

class RobotSteering(QObject):

    def __init__(self, parent, plugin_context):
        super(RobotSteering, self).__init__(parent)
        self.setObjectName('RobotSteering')

        self.publisher_ = None
        main_window = plugin_context.main_window()
        self.widget_ = QDockWidget(main_window)

        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'RobotSteering.ui')
        loadUi(ui_file, self.widget_)
        self.widget_.setObjectName('rosgui_robot_steering_%d' % plugin_context.serial_number())
        if plugin_context.serial_number() != 1:
            self.widget_.setWindowTitle(self.widget_.windowTitle() + (' (%d)' % plugin_context.serial_number()))
        main_window.addDockWidget(Qt.RightDockWidgetArea, self.widget_)

        # trigger deleteLater for plugin when widget is closed
        self.widget_.installEventFilter(self)

        self.widget_.topic_line_edit.textChanged.connect(self.__on_topic_changed)

        self.widget_.x_linear_slider.valueChanged.connect(self.__on_parameter_changed)
        self.widget_.z_angular_slider.valueChanged.connect(self.__on_parameter_changed)

        self.widget_.increase_x_linear_push_button.pressed.connect(self.__on_increase_x_linear_pressed)
        self.widget_.decrease_x_linear_push_button.pressed.connect(self.__on_decrease_x_linear_pressed)
        self.widget_.increase_z_angular_push_button.pressed.connect(self.__on_increase_z_angular_pressed)
        self.widget_.decrease_z_angular_push_button.pressed.connect(self.__on_decrease_z_angular_pressed)
        self.widget_.stop_push_button.pressed.connect(self.__on_stop_pressed)

        self.shortcut_w_ = QShortcut(Qt.Key_W, self.widget_)
        self.shortcut_w_.setContext(Qt.ApplicationShortcut)
        self.shortcut_w_.activated.connect(self.__on_increase_x_linear_pressed)
        self.shortcut_s_ = QShortcut(Qt.Key_S, self.widget_)
        self.shortcut_s_.setContext(Qt.ApplicationShortcut)
        self.shortcut_s_.activated.connect(self.__on_decrease_x_linear_pressed)
        self.shortcut_a_ = QShortcut(Qt.Key_A, self.widget_)
        self.shortcut_a_.setContext(Qt.ApplicationShortcut)
        self.shortcut_a_.activated.connect(self.__on_increase_z_angular_pressed)
        self.shortcut_d_ = QShortcut(Qt.Key_D, self.widget_)
        self.shortcut_d_.setContext(Qt.ApplicationShortcut)
        self.shortcut_d_.activated.connect(self.__on_decrease_z_angular_pressed)

        self.shortcut_shift_w_ = QShortcut(Qt.SHIFT + Qt.Key_W, self.widget_)
        self.shortcut_shift_w_.setContext(Qt.ApplicationShortcut)
        self.shortcut_shift_w_.activated.connect(self.__on_strong_increase_x_linear_pressed)
        self.shortcut_shift_s_ = QShortcut(Qt.SHIFT + Qt.Key_S, self.widget_)
        self.shortcut_shift_s_.setContext(Qt.ApplicationShortcut)
        self.shortcut_shift_s_.activated.connect(self.__on_strong_decrease_x_linear_pressed)
        self.shortcut_shift_a_ = QShortcut(Qt.SHIFT + Qt.Key_A, self.widget_)
        self.shortcut_shift_a_.setContext(Qt.ApplicationShortcut)
        self.shortcut_shift_a_.activated.connect(self.__on_strong_increase_z_angular_pressed)
        self.shortcut_shift_d_ = QShortcut(Qt.SHIFT + Qt.Key_D, self.widget_)
        self.shortcut_shift_d_.setContext(Qt.ApplicationShortcut)
        self.shortcut_shift_d_.activated.connect(self.__on_strong_decrease_z_angular_pressed)

        self.shortcut_space_ = QShortcut(Qt.Key_Space, self.widget_)
        self.shortcut_space_.setContext(Qt.ApplicationShortcut)
        self.shortcut_space_.activated.connect(self.__on_stop_pressed)

        # timer to consecutively send twist messages
        self.timerUpdatePlot = QTimer(self)
        self.timerUpdatePlot.timeout.connect(self.__on_parameter_changed)
        self.timerUpdatePlot.start(100)

    @Slot(str)
    def __on_topic_changed(self, topic):
        topic = str(topic)
        self.__unregisterPublisher()
        self.publisher_ = rospy.Publisher(topic, Twist)

    def __on_parameter_changed(self):
        self.__send_twist(self.widget_.x_linear_slider.value() / 1000.0, self.widget_.z_angular_slider.value() / 1000.0)

    def __send_twist(self, x_linear, z_angular):
        if self.publisher_ is None:
            return
        twist = Twist()
        twist.linear.x = x_linear
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = z_angular
        self.publisher_.publish(twist)

    def __on_increase_x_linear_pressed(self):
        self.widget_.x_linear_slider.setValue(self.widget_.x_linear_slider.value() + self.widget_.x_linear_slider.singleStep())

    def __on_decrease_x_linear_pressed(self):
        self.widget_.x_linear_slider.setValue(self.widget_.x_linear_slider.value() - self.widget_.x_linear_slider.singleStep())

    def __on_increase_z_angular_pressed(self):
        self.widget_.z_angular_slider.setValue(self.widget_.z_angular_slider.value() + self.widget_.z_angular_slider.singleStep())

    def __on_decrease_z_angular_pressed(self):
        self.widget_.z_angular_slider.setValue(self.widget_.z_angular_slider.value() - self.widget_.z_angular_slider.singleStep())

    def __on_strong_increase_x_linear_pressed(self):
        self.widget_.x_linear_slider.setValue(self.widget_.x_linear_slider.value() + self.widget_.x_linear_slider.pageStep())

    def __on_strong_decrease_x_linear_pressed(self):
        self.widget_.x_linear_slider.setValue(self.widget_.x_linear_slider.value() - self.widget_.x_linear_slider.pageStep())

    def __on_strong_increase_z_angular_pressed(self):
        self.widget_.z_angular_slider.setValue(self.widget_.z_angular_slider.value() + self.widget_.z_angular_slider.pageStep())

    def __on_strong_decrease_z_angular_pressed(self):
        self.widget_.z_angular_slider.setValue(self.widget_.z_angular_slider.value() - self.widget_.z_angular_slider.pageStep())

    def __on_stop_pressed(self):
        self.widget_.x_linear_slider.setValue(0)
        self.widget_.z_angular_slider.setValue(0)

    def __unregisterPublisher(self):
        if self.publisher_ is not None:
            self.publisher_.unregister()
            self.publisher_ = None

    def eventFilter(self, obj, event):
        if obj == self.widget_ and event.type() == QEvent.Close:
            # TODO: ignore() should not be necessary when returning True
            event.ignore()
            self.deleteLater()
            return True
        return QObject.eventFilter(self, obj, event)

    def close_plugin(self):
        self.__unregisterPublisher()
        self.widget_.close()
        self.widget_.deleteLater()

    def save_settings(self, global_settings, perspective_settings):
        topic = self.widget_.topic_line_edit.text()
        perspective_settings.set_value('topic', topic)

    def restore_settings(self, global_settings, perspective_settings):
        topic = perspective_settings.value('topic', '/cmd_vel')
        self.widget_.topic_line_edit.setText(topic)
