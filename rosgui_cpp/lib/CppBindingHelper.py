import os, sys

from rosgui.QtBindingHelper import QT_BINDING
from rosgui.RosPackageHelper import get_package_path

try:
    if QT_BINDING == 'pyside':
        # append "rosgui_cpp_shiboken/lib" folder to module search path
        shiboken_path = get_package_path('rosgui_cpp_shiboken')
        sys.path.append(os.path.join(shiboken_path, 'lib'))
        import librosgui_cpp_shiboken
        sys.modules['rosgui_cpp'] = librosgui_cpp_shiboken

    elif QT_BINDING == 'pyqt':
        # append "rosgui_cpp_sip/lib" folder to module search path
        sip_path = get_package_path('rosgui_cpp_sip')
        sys.path.append(os.path.join(sip_path, 'lib'))
        import librosgui_cpp_sip
        sys.modules['rosgui_cpp'] = librosgui_cpp_sip

    else:
        raise ImportError()

except ImportError:
    raise ImportError('Could not import "%s" bindings of rosgui_cpp library - so C++ plugins will not be available' % QT_BINDING)
