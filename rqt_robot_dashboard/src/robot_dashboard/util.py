import roslib;roslib.load_manifest('robot_dashboard')
import rospy

from QtGui import QMessageBox, QIcon, QPixmap
from PIL.ImageQt import ImageQt

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

def make_icon(image, mode = 0):
    """Helper function to convert a PIL Image to a QIcon.

    :param image: Image to convert.
    :type image: PIL.Image.Image
    :param mode: The mode of the QIcon.
    :type mode: int
    """
    qim = ImageQt(image)
    icon = QIcon()
    icon.addPixmap(QPixmap.fromImage(qim), mode)
    return icon

