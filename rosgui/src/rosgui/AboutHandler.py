import os, platform, sys

from QtBindingHelper import QT_BINDING, QT_BINDING_VERSION
from QtCore import QObject, qVersion
from QtGui import QMessageBox

from RosPackageHelper import get_package_path

class AboutHandler(QObject):

    def __init__(self, parent=None):
        super(AboutHandler, self).__init__(parent)
        self.parent_ = parent

    def show(self):
        # append folder of 'rosgui_cpp/lib' to module search path
        sys.path.append(os.path.realpath(os.path.join(get_package_path('rosgui_cpp'), 'lib')))
        try:
            import CppBindingHelper #@UnusedImport
            from rosgui_cpp import rosgui_cpp
        except ImportError:
            rosgui_cpp = None

        _rospkg_version = None
        try:
            import rospkg
            _rospkg_version = getattr(rospkg, '__version__', '&lt; 0.2.4')
        except ImportError:
            pass

        logo = os.path.realpath(os.path.join(os.path.dirname(__file__), '..', '..', 'icons', 'ros_org_vertical.png'))
        text = '<img src="%s" width="56" height="200" style="float: left;"/>' % logo

        text += '<h3 style="margin-top: 1px;">%s</h3>' % self.tr('ROS GUI')

        text += '<p>%s %s</p>' % (self.tr('ROS GUI is a framework for graphical user interfaces.'), self.tr('It is extensible with plugins which can be written in either Python or C++.'))
        text += '<p>%s</p>' % (self.tr('Please see the <a href="%s">Wiki</a> for more information on ROS GUI and available plugins.' % 'http://www.ros.org/wiki/ros_gui'))

        text += '<p>%s: ' % self.tr('Utilized libraries:')

        text += 'Python %s, ' % platform.python_version()

        if _rospkg_version is not None:
            text += 'rospkg %s, ' % _rospkg_version
        else:
            text += '%s, ' % self.tr('rospkg not found - using roslib')

        if QT_BINDING == 'pyside':
            text += 'PySide %s, ' % QT_BINDING_VERSION
        elif QT_BINDING == 'pyqt':
            text += 'PyQt %s, ' % QT_BINDING_VERSION

        text += 'Qt %s, ' % qVersion()

        if rosgui_cpp is not None:
            if QT_BINDING == 'pyside':
                text += '%s' % (self.tr('%s C++ bindings available') % 'Shiboken')
            elif QT_BINDING == 'pyqt':
                text += '%s' % (self.tr('%s C++ bindings available') % 'SIP')
            else:
                text += '%s' % self.tr('C++ bindings available')
        else:
            text += '%s' % self.tr('no C++ bindings found - no C++ plugins available')

        text += '.</p>'

        QMessageBox.about(self.parent_, self.tr('About ROS GUI'), text)
