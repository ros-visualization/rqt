# Copyright (c) 2011, Dirk Thomas, Dorian Scholz, TU Darmstadt
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

import os

from catkin_pkg.package import parse_package

from rospkg import RosPack
from rospkg.common import MANIFEST_FILE, PACKAGE_FILE
from rospkg.manifest import parse_manifest_file

from .ros_plugin_provider import RosPluginProvider

class RospkgPluginProvider(RosPluginProvider):

    """`RosPluginProvider` using rospkg."""

    def __init__(self, export_tag, base_class_type):
        super(RospkgPluginProvider, self).__init__(export_tag, base_class_type)
        self.setObjectName('RospkgPluginProvider')

    def _find_plugins(self, export_tag):
        plugins = []
        r = RosPack()
        for package_name in r.list():
            path = r.get_path(package_name)
            manifest_path = os.path.join(path, MANIFEST_FILE)
            if os.path.isfile(manifest_path):
                try:
                    manifest = parse_manifest_file(path, MANIFEST_FILE)
                except InvalidManifest as e:
                    qWarning('Could not parse manifest "%s":\n%s' % (manifest_path, e))
                    continue
                exports = manifest.get_export(export_tag, 'plugin')
                for export in exports:
                    plugins.append([package_name, str(export)])
                continue
            package_path = os.path.join(path, PACKAGE_FILE)
            if os.path.isfile(package_path):
                try:
                    package = parse_package(package_path)
                except InvalidPackage as e:
                    qWarning('Could not parse package "%s":\n%s' % (package_path, e))
                    continue
                for export in package.exports:
                    if export.tagname != export_tag or 'plugin' not in export.attributes:
                        continue
                    plugin_path = export.attributes['plugin']
                    if plugin_path.startswith('${prefix}/'):
                        plugin_path = os.path.join(path, plugin_path[10:])
                    plugins.append([package_name, plugin_path])
                continue
        return plugins
