# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# * Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above
# copyright notice, this list of conditions and the following
# disclaimer in the documentation and/or other materials provided
# with the distribution.
# * Neither the name of Willow Garage, Inc. nor the names of its
# contributors may be used to endorse or promote products derived
# from this software without specific prior written permission.
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

import os
import shelve
import time

from .ros_plugin_provider import RosPluginProvider


class RosPluginProviderCache():

    """Cache persisting the plugin list to speed up start-up time."""

    def __init__(self):
        self._cache_file = os.path.expanduser('~/.config/ros.org/rqt-plugin-cache')
        self._loaded = None
        self._max_age = 60 * 60 * 24  # one day

    def load(self):
        if os.path.exists(self._cache_file):
            data = shelve.open(self._cache_file)
            ts = data['timestamp']
            if ts + self._max_age >= time.time():
                self._loaded = ts
                RosPluginProvider._cached_plugins = data['plugins']
                print('RosPluginProviderCache.load() using cached list of plugins')
            data.close()

    def save(self):
        data = shelve.open(self._cache_file)
        # only save data when not loaded before or not modified in between but has changes
        if self._loaded is None or (self._loaded == data['timestamp'] and RosPluginProvider._cached_plugins != data['plugins']):
            data['plugins'] = RosPluginProvider._cached_plugins
            data['timestamp'] = time.time()
            print('RosPluginProviderCache.save() written list of plugins to cache')
        data.close()
