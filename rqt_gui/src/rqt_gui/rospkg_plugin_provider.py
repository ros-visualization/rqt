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

from python_qt_binding.QtCore import qWarning

from rospkg import RosPack
from rospkg.common import MANIFEST_FILE, PACKAGE_FILE
from rospkg.manifest import parse_manifest_file, InvalidManifest

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
            package_path = r.get_path(package_name)
            manifest_file_path = os.path.join(package_path, MANIFEST_FILE)
            if os.path.isfile(manifest_file_path):
                try:
                    manifest = parse_manifest_file(package_path, MANIFEST_FILE)
                except InvalidManifest as e:
                    qWarning('Could not parse manifest "%s":\n%s' % (manifest_file_path, e))
                    continue
                exports = manifest.get_export(export_tag, 'plugin')
                for export in exports:
                    plugins.append([package_name, str(export)])
                continue

            package_file_path = os.path.join(package_path, PACKAGE_FILE)
            if os.path.isfile(package_file_path):
                # only try to import catkin if a PACKAGE_FILE is found
                try:
                    from catkin_pkg.package import parse_package, InvalidPackage
                except ImportError as e:
                    qWarning('Package "%s" has a package file, but import of parser failed:\n%s' % (package_path, e))
                    continue
                try:
                    package = parse_package(package_file_path)
                except InvalidPackage as e:
                    qWarning('Could not parse package file "%s":\n%s' % (package_file_path, e))
                    continue
                for export in package.exports:
                    if export.tagname != export_tag or 'plugin' not in export.attributes:
                        continue
                    plugin_xml_path = export.attributes['plugin']
                    plugin_xml_path = plugin_xml_path.replace('${prefix}', package_path)
                    plugins.append([package_name, plugin_xml_path])
                continue
        return plugins
