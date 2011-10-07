import os, sys

from rosgui.QtBindingHelper import QT_BINDING
from rosgui.RosPackageHelper import get_package_path

try:
    if QT_BINDING == 'pyside':
        # append "rosgui_cpp_shiboken/lib" folder to module search path
        shiboken_path = get_package_path('rosgui_cpp_shiboken')
        sys.path.append(os.path.join(shiboken_path, 'lib'))
        from librosgui_cpp_shiboken import rosgui_cpp
    elif QT_BINDING == 'pyqt':
        # append "rosgui_cpp_sip/lib" folder to module search path
        sip_path = get_package_path('rosgui_cpp_sip')
        sys.path.append(os.path.join(sip_path, 'lib'))
        from librosgui_cpp_sip import rosgui_cpp
    else:
        raise ImportError()
except ImportError:
    raise ImportError('Could not import "%s" bindings of rosgui_cpp library - so C++ plugins will not be available' % QT_BINDING)

ROSGUI_CPP_BINDING = rosgui_cpp

