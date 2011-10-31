import os, sys, traceback

from rosgui.QtBindingHelper import QT_BINDING
from QtCore import qWarning
from rosgui.RosPackageHelper import get_package_path

try:
    if QT_BINDING == 'pyside':
        # append "rosgui_cpp_shiboken/lib" folder to module search path
        shiboken_path = get_package_path('rosgui_cpp_shiboken')
        sys.path.append(os.path.join(shiboken_path, 'lib'))
        import librosgui_cpp_shiboken
        rosgui_cpp = librosgui_cpp_shiboken.rosgui_cpp

    elif QT_BINDING == 'pyqt':
        # append "rosgui_cpp_sip/lib" folder to module search path
        sip_path = get_package_path('rosgui_cpp_sip')
        sys.path.append(os.path.join(sip_path, 'lib'))
        import librosgui_cpp_sip
        rosgui_cpp = librosgui_cpp_sip.rosgui_cpp

    else:
        raise ImportError('Qt binding name "%s" is unknown.' % QT_BINDING)

except ImportError:
    rosgui_cpp = None
    qWarning('Could not import "%s" bindings of rosgui_cpp library - so C++ plugins will not be available:\n%s' % (QT_BINDING, traceback.format_exc()))
