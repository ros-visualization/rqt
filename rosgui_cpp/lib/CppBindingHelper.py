# Copyright (c) 2011, Dirk Thomas, TU Darmstadt
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
