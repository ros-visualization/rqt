# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Loads plugins.
"""

import qt_gui.qt_binding_helper  # @UnusedImport

from QtCore import qWarning
import sys
import traceback

import roslib
import rospkg


def load_plugins():
    """
    Finds all rqt_bag plugins.

    @return: a list of plugins
    @rtype:  list of functions which return tuples of (MessageView, TimelineRenderer, list of: message type or '*')
    """
    plugins = []

    rospack = rospkg.RosPack()
    to_check = rospack.get_depends_on('rqt_bag', implicit=False)

    for pkg in to_check:
        manifest = rospack.get_manifest(pkg)
        plugin_module_names = manifest.get_export('rqt_bag', 'plugin')
        if not plugin_module_names:
            continue
        elif len(plugin_module_names) != 1:
            qWarning("Cannot load plugin [%s]: invalid 'plugin' attribute" % (pkg), file=sys.stderr)
            continue
        plugin_module_name = plugin_module_names[0]

        try:
            # Load that package's namespace
            roslib.load_manifest(pkg)

            # Import specified plugin module
            plugin_module = __import__(plugin_module_name)
            for sub_module in plugin_module_name.split('.')[1:]:
                plugin_module = getattr(plugin_module, sub_module)

            # Retrieve the function
            plugins_func = None
            try:
                plugins_func = getattr(plugin_module, 'get_rqt_bag_plugins')
            except AttributeError:
                pass

            if plugins_func:
                plugins.extend(plugins_func())
            else:
                qWarning("Cannot load plugin [%s]: no 'get_rqt_bag_plugins' attribute" % (plugin_module_name), file=sys.stderr)

        except Exception:
            qWarning("Unable to load plugin [%s] from package [%s]:\n%s" % (plugin_module_name, pkg, traceback.format_exc()), file=sys.stderr)
    return plugins
