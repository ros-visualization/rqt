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
#
# Author: Isaac Saito, Ze'ev Klapow

import threading
import time

import dynamic_reconfigure
import rospy

class ParamUpdater(threading.Thread):
    #TODO(Isaac) Modify variable names to the ones that make more intuition.
     
    def __init__(self, reconf):
        super(ParamUpdater, self).__init__()
        self.setDaemon(True)

        self._reconf = reconf
        self._cv = threading.Condition()
        self._pending_config = {}
        self._last_pending = None
        self._stop_flag = False

    def run(self):
        last_commit = None

        while not self._stop_flag:
            if last_commit >= self._last_pending:
                    with self._cv:
                        self._cv.wait()

            if self._stop_flag:  
                return

            last_commit = time.time()
            update = self._pending_config.copy()
            self._pending_config = {}

            try:
                updated = self._reconf.update_configuration(update)
            except rospy.ServiceException as ex:
                rospy.logdebug('Could not update configuration')
            except Exception as exc:
                raise exc

    def update(self, config):
        with self._cv:
            for name, value in config.items():
                self._pending_config[name] = value

            self._last_pending = time.time()

            self._cv.notify()

    def stop(self):
        self._stop_flag = True 
        with self._cv:
            self._cv.notify()
