# Copyright (c) 2026, Martin Pecka, Open Robotics
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
#   * Neither the name of the copyright holder nor the names of its
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

# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Open Robotics

import os
from pathlib import Path
import shutil
import sys

BASE_DIR = Path(os.path.dirname(os.path.abspath(__file__)))

PACKAGE_NAME = 'test_rqt_gui_py'
PLUGIN_CATEGORY = 'rqt_gui'
PACKAGE_XML = BASE_DIR / 'test_package.xml'
PLUGIN_XMLs = [BASE_DIR / 'test_plugins.xml']


def pytest_configure(config):
    # Get the path to the text file in the same directory as conftest.py
    mock_dir: Path = config.cache.mkdir('test_rqt_gui_py')

    # This code reflects what pluginlib_enable_plugin_testing from pluginlib does:
    # It prepares a fake directory and file structure that is appended to the current
    # environment to allow usage of pluginlib plugins defined just for the tests.

    # Each "PACKAGE_NAME" goes to its own folder
    prefix = mock_dir / 'install' / f'{PACKAGE_NAME}__{PLUGIN_CATEGORY}'
    prefix.mkdir(parents=True, exist_ok=True)
    ament_resource_index = prefix / 'share' / 'ament_index' / 'resource_index'
    ament_resource_index.mkdir(parents=True, exist_ok=True)

    # Install package.xml, renaming to just `package.xml` if needed
    fake_install_dir = prefix / 'share' / PACKAGE_NAME
    fake_install_dir.mkdir(parents=True, exist_ok=True)
    shutil.copy(PACKAGE_XML, fake_install_dir / 'package.xml')

    # Add blank file indicating existence of package in the ament index
    (ament_resource_index / 'packages').mkdir(parents=True, exist_ok=True)
    (ament_resource_index / 'packages' / PACKAGE_NAME).touch()

    # Write the plugin descriptions in share/
    ament_index_plugin_content = ''
    for plugin_xml in PLUGIN_XMLs:
        shutil.copy(plugin_xml, fake_install_dir / plugin_xml.name)
        ament_index_plugin_content += f'\nshare/{PACKAGE_NAME}/{plugin_xml.name}'
    # Get rid of leading newline
    ament_index_plugin_content = ament_index_plugin_content.strip()

    # Write one file that says where all of this package's plugin description files are
    (ament_resource_index / f'{PLUGIN_CATEGORY}__pluginlib__plugin').mkdir(
        parents=True, exist_ok=True)
    with open(
            ament_resource_index / f'{PLUGIN_CATEGORY}__pluginlib__plugin' / PACKAGE_NAME,
            'w') as f:
        f.write(ament_index_plugin_content)

    ament_prefix_path = os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep)
    ament_prefix_path.insert(0, str(prefix))
    os.environ['AMENT_PREFIX_PATH'] = os.pathsep.join(ament_prefix_path)

    sys.path.insert(0, str(BASE_DIR))
    pythonpath = os.environ.get('PYTHONPATH', '').split(os.pathsep)
    pythonpath.insert(0, str(BASE_DIR))
    os.environ['PYTHONPATH'] = os.pathsep.join(pythonpath)

    print(f'PYTHONPATH += {str(BASE_DIR)}')
    print(f'AMENT_PREFIX_PATH += {str(prefix)}')
