# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Isaac Saito

import os

from ament_index_python import get_resources

from rclpy import logging


class RqtRoscommUtil(object):
    _logger = logging.get_logger('RqtRoscommUtil')

    # TODO(mlautman): Port load_parameters from ROS 1 (ROSLaunchRunner) to
    # ROS 2 once an equivalent for the parameter-server bulk-load API exists.
    # The previous ROS 1 implementation was removed; see git history.

    @staticmethod
    def iterate_packages(subdir):
        """
        Iterate packages that contain the given subdir.

        This method is generalizing rosmsg.iterate_packages.

        @param subdir: eg. 'launch', 'msg', 'srv', 'action'
        @type subdir: str
        @raise ValueError:
        """
        if subdir is None or subdir == '':
            raise ValueError(f'Invalid package subdir = {subdir}')

        packages_map = get_resources('packages')
        for package_name, package_path in packages_map.items():
            package_path = os.path.join(package_path, 'share', package_name, subdir)
            RqtRoscommUtil._logger.debug(
                f'package:\t{package_name} dir:\t{package_path}')
            if os.path.isdir(package_path):
                yield package_name, package_path
