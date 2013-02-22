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
# Author: Isaac Saito

import roslaunch
import rospy


class RoslaunchAccesor(object):

    @staticmethod
    def load_parameters(config, caller_id):
        """
        Load parameters onto the parameter server.

        Copied from ROSLaunchRunner.

        @type config: roslaunch.config.ROSLaunchConfig
        @raise roslaunch.RLException:
        """

        # XMLRPC proxy for communicating with master, 'xmlrpclib.ServerProxy'
        param_server = config.master.get()

        p = None
        try:
            # multi-call style xmlrpc
            param_server_multi = config.master.get_multi()

            # clear specified parameter namespaces
            # #2468 unify clear params to prevent error
            for p in roslaunch.launch._unify_clear_params(config.clear_params):
                if param_server.hasParam(caller_id, p)[2]:
                    param_server_multi.deleteParam(caller_id, p)
            r = param_server_multi()
            for code, msg, _ in r:
                if code != 1:
                    raise roslaunch.RLException("Failed to clear parameter: " +
                                                "%s" % (msg))

            # multi-call objects are not reusable
            param_server_multi = config.master.get_multi()
            for p in config.params.itervalues():
                # suppressing this as it causes too much spam
                # printlog("setting parameter [%s]"%p.key)
                param_server_multi.setParam(caller_id, p.key, p.value)
            r = param_server_multi()
            for code, msg, _ in r:
                if code != 1:
                    raise roslaunch.RLException("Failed to set parameter: " +
                                                "%s" % (msg))
        except roslaunch.RLException:
            raise
        except Exception, e:
            print("load_parameters: unable to set params (last param was " +
                  "[%s]): %s" % (p, e))
            raise  # re-raise as this is fatal
        rospy.loginfo("... load_parameters complete")
