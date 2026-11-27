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

from python_qt_binding.QtWidgets import QApplication

from rqt_gui_py.plugin import Plugin

TEST_PLUGIN_ARGV_PROPERTY = 'test_rqt_gui_py_argv'
TEST_PLUGIN_SIM_TIME_PROPERTY = 'test_rqt_gui_py_sim_time'
TEST2_PLUGIN_ARGV_PROPERTY = 'test_rqt_gui_py_argv2'
TEST2_PLUGIN_SIM_TIME_PROPERTY = 'test_rqt_gui_py_sim_time2'


class Test(Plugin):

    __test__ = False

    def __init__(self, context):
        super().__init__(context)
        self._node = context.node
        app = QApplication.instance()
        if app is not None:
            prefix = ''
            for i in range(len(context.argv())):
                arg = context.argv()[i]
                if arg == '--prefix' and i + 1 < len(context.argv()):
                    prefix = context.argv()[i + 1]

            app.setProperty(prefix + TEST_PLUGIN_ARGV_PROPERTY, context.argv())
            if self._node is not None:
                app.setProperty(
                    prefix + TEST_PLUGIN_SIM_TIME_PROPERTY,
                    self._node.get_parameter('use_sim_time').value)


class Te2st(Plugin):

    __test__ = False

    def __init__(self, context):
        super().__init__(context)
        self._node = context.node
        app = QApplication.instance()
        if app is not None:
            app.setProperty(TEST2_PLUGIN_ARGV_PROPERTY, context.argv())
            if self._node is not None:
                app.setProperty(
                    TEST2_PLUGIN_SIM_TIME_PROPERTY,
                    self._node.get_parameter('use_sim_time').value)
