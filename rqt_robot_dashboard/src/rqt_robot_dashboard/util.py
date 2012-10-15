import roslib;roslib.load_manifest('rqt_robot_dashboard')
import rospy

from QtGui import  QIcon, QMessageBox, QPainter, QPixmap

def dashinfo(msg, obj, title = 'Info'):
    """Logs a message with ``rospy.loginfo`` and displays a ``QMessageBox`` to the user

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

def dashwarn(msg, obj, title = 'Warning'):
    """Logs a message with ``rospy.logwarn`` and displays a ``QMessageBox`` to the user

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

def dasherr(msg, obj, title = 'Error'):
    """Logs a message with ``rospy.logerr`` and displays a ``QMessageBox`` to the user

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

def make_icon(image_list, mode = QIcon.Normal, state = QIcon.On):
    """Helper function to create QIcons from lists of image files

    :param image_list: list of paths to Images that will be sequentially layered into an icon.
    :type image: str
    :param mode: The mode of the QIcon.
    :type mode: int
    :param state: the state of the QIcon.
    :type state: int
    """
    if type(image_list) is not list:
        image_list = [image_list]

    icon_pixmap = QPixmap()
    icon_pixmap.load(image_list[0])
    painter = QPainter(icon_pixmap)
    for item in image_list[1:]:
        painter.drawPixmap(0, 0, QPixmap(item))
    icon = QIcon()
    icon.addPixmap(icon_pixmap, mode, state)
    painter.end()
    return icon

