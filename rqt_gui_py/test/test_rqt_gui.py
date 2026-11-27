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

"""
Unit test for rqt_gui_py.

This unit test runs rqt_gui as if from command-line and passes various combinations
of plugin and ROS args. It tests whether both plugin args and ROS args are correctly
interpreted and passed.
"""

import multiprocessing as mp
import os
import sys
import time

import pytest

from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtWidgets import QApplication, QMainWindow

from rqt_gui.main import Main

TEST_PLUGIN_NAME = 'test_rqt_gui_py/_test'
# the properties that test_plugin.py's initPlugin() sets on the QApplication instance
TEST_PLUGIN_ARGV_PROPERTY = 'test_rqt_gui_py_argv'
TEST_PLUGIN_SIM_TIME_PROPERTY = 'test_rqt_gui_py_sim_time'

TEST2_PLUGIN_NAME = 'test_rqt_gui_py/_2test'
TEST2_PLUGIN_ARGV_PROPERTY = 'test_rqt_gui_py_argv2'
TEST2_PLUGIN_SIM_TIME_PROPERTY = 'test_rqt_gui_py_sim_time2'


def qt_wait(app: QApplication, cond, timeout: float, sleep: float = 0.01, msg: str | None = None):
    deadline = time.monotonic() + timeout
    while not cond() and time.monotonic() < deadline:
        if msg is not None:
            print(msg, file=sys.stderr)
        app.processEvents()
        time.sleep(sleep)
    if not cond:
        raise TimeoutError(f'Condition timed out after {timeout} seconds')


class MainClass(Main):

    def __init__(self, test_cb, filename=None, settings_filename='rqt_gui'):
        super().__init__(filename, settings_filename)
        self.app: QApplication | None = None
        self.test_cb = test_cb
        self.closed_cleanly = False
        self.exc: BaseException | None = None

    def create_application(self, argv):
        self.app = super().create_application(argv)

        def cb():
            try:
                def get_main_windows():
                    return [
                        w for w in QApplication.topLevelWidgets() if isinstance(w, QMainWindow)
                    ]
                qt_wait(self.app, lambda: len(get_main_windows()) > 0, 10.0, sleep=0.1,
                        msg='Waiting for window to open')
                main_window = get_main_windows()[0]

                qt_wait(self.app, lambda: len(main_window.objectName()) > 0, 10.0, sleep=0.1,
                        msg='Waiting for window to be ready')

                self.test_cb(self.app, main_window)
                self.shutdown()
            except BaseException as e:
                self.exc = e
                QApplication.closeAllWindows()

        QTimer.singleShot(500, cb)
        return self.app

    def shutdown(self):
        if QApplication.topLevelWidgets():
            self.closed_cleanly = True
        QApplication.closeAllWindows()


def _test_cb_just_opens_and_closes(app: QApplication, main_window: QMainWindow):
    """Test that the test windows does not load the plugin by default."""
    assert main_window.objectName() == 'MainWindow'
    # without asking for the plugin explicitly, it must not be loaded
    assert app.property(TEST_PLUGIN_ARGV_PROPERTY) is None


def _test_cb_loads_test_plugin(app: QApplication, main_window: QMainWindow):
    """Test that zero arguments can be specified."""
    assert main_window.objectName() == 'MainWindow'
    assert app.property(TEST_PLUGIN_ARGV_PROPERTY) is not None, \
        'The test plugin %s was not loaded by rqt_gui.' % TEST_PLUGIN_NAME
    assert app.property(TEST_PLUGIN_SIM_TIME_PROPERTY) is not None, \
        'The test plugin %s was loaded but did not receive its ROS node.' % TEST_PLUGIN_NAME
    assert app.property(TEST_PLUGIN_ARGV_PROPERTY) == [], \
        'The plugin received unexpected arguments: %s' % app.property(TEST_PLUGIN_ARGV_PROPERTY)
    assert app.property(TEST_PLUGIN_SIM_TIME_PROPERTY) is False, \
        'use_sim_time parameter was not set to False.'


def _test_cb_sets_ros_parameter(app: QApplication, main_window: QMainWindow):
    """Test that only ROS arguments can be specified."""
    assert main_window.objectName() == 'MainWindow'
    assert app.property(TEST_PLUGIN_ARGV_PROPERTY) is not None, \
        'The test plugin %s was not loaded by rqt_gui.' % TEST_PLUGIN_NAME
    assert app.property(TEST_PLUGIN_SIM_TIME_PROPERTY) is not None, \
        'The test plugin %s was loaded but did not receive its ROS node.' % TEST_PLUGIN_NAME
    assert app.property(TEST_PLUGIN_ARGV_PROPERTY) == [], \
        'The plugin received unexpected arguments: %s' % app.property(TEST_PLUGIN_ARGV_PROPERTY)
    assert app.property(TEST_PLUGIN_SIM_TIME_PROPERTY) is True, \
        'use_sim_time parameter was not set to True.'


def _test_cb_passes_plugin_args(app: QApplication, main_window: QMainWindow):
    """Test that only plugin params can be specified."""
    assert main_window.objectName() == 'MainWindow'
    assert app.property(TEST_PLUGIN_ARGV_PROPERTY) is not None, \
        'The test plugin %s was not loaded by rqt_gui.' % TEST_PLUGIN_NAME
    assert app.property(TEST_PLUGIN_SIM_TIME_PROPERTY) is not None, \
        'The test plugin %s was loaded but did not receive its ROS node.' % TEST_PLUGIN_NAME
    assert app.property(TEST_PLUGIN_ARGV_PROPERTY) == ['-p', '-q', '--raw'], \
        'The plugin received unexpected arguments: %s' % app.property(TEST_PLUGIN_ARGV_PROPERTY)
    assert app.property(TEST_PLUGIN_SIM_TIME_PROPERTY) is False, \
        'use_sim_time parameter was not set to False.'


def _test_cb_passes_plugin_and_ros_args(app: QApplication, main_window: QMainWindow):
    """Test that both ROS and plugin params can be specified."""
    assert main_window.objectName() == 'MainWindow'
    assert app.property(TEST_PLUGIN_ARGV_PROPERTY) is not None, \
        'The test plugin %s was not loaded by rqt_gui.' % TEST_PLUGIN_NAME
    assert app.property(TEST_PLUGIN_SIM_TIME_PROPERTY) is not None, \
        'The test plugin %s was loaded but did not receive its ROS node.' % TEST_PLUGIN_NAME
    assert app.property(TEST_PLUGIN_ARGV_PROPERTY) == ['-p', '-q', '--raw'], \
        'The plugin received unexpected arguments: %s' % app.property(TEST_PLUGIN_ARGV_PROPERTY)
    assert app.property(TEST_PLUGIN_SIM_TIME_PROPERTY) is True, \
        'use_sim_time parameter was not set to True.'


def _insert_plugin(pid, queue, plugin, argv):
    main = Main()
    argv = [
        os.path.abspath(__file__),
        '--command-pid', str(pid),
        '--command-start-plugin', plugin
    ] + (argv if argv else [])
    ret = main.main(argv=argv)
    assert ret == 0, 'rqt_gui command returned non-zero exit code %d' % ret
    queue.put((ret,))


def insert_plugin(app: QApplication, plugin, check_property, argv=None):
    pid = app.applicationPid()

    ctx = mp.get_context('spawn')
    queue = ctx.Queue()
    proc = ctx.Process(target=_insert_plugin, args=(pid, queue, plugin, argv))
    proc.start()

    qt_wait(app, lambda: proc.is_alive() is False, 10.0, sleep=0.2,
            msg='Waiting for command to finish')

    proc.join(timeout=1)
    assert proc.is_alive() is False, 'Command timed out'
    ret, = queue.get_nowait()
    assert ret == 0, 'rqt_gui command returned non-zero exit code %d' % ret

    qt_wait(app, lambda: app.property(check_property) is not None, 10.0, sleep=0.2,
            msg='Waiting for dynamically loaded plugin')


def _test_cb_runtime_plugins_get_ros_params(app: QApplication, main_window: QMainWindow):
    """Test that ROS parameters are correctly passed to plugins added during runtime."""
    assert main_window.objectName() == 'MainWindow'
    assert app.property(TEST_PLUGIN_ARGV_PROPERTY) is None, \
        'The test plugin %s was loaded early by rqt_gui.' % TEST_PLUGIN_NAME
    assert app.property(TEST_PLUGIN_SIM_TIME_PROPERTY) is None, \
        'The test plugin %s was loaded early by rqt_gui.' % TEST_PLUGIN_NAME

    insert_plugin(app, TEST_PLUGIN_NAME, TEST_PLUGIN_ARGV_PROPERTY)

    assert app.property(TEST_PLUGIN_ARGV_PROPERTY) == [], \
        'The plugin received unexpected arguments: %s' % app.property(TEST_PLUGIN_ARGV_PROPERTY)
    assert app.property(TEST_PLUGIN_SIM_TIME_PROPERTY) is True, \
        'use_sim_time parameter was not set to True.'

    # Test that even the second added plugin receives the ROS args
    insert_plugin(app, TEST2_PLUGIN_NAME, TEST2_PLUGIN_ARGV_PROPERTY)

    assert app.property(TEST2_PLUGIN_ARGV_PROPERTY) == [], \
        'The plugin received unexpected arguments: %s' % app.property(TEST2_PLUGIN_ARGV_PROPERTY)
    assert app.property(TEST2_PLUGIN_SIM_TIME_PROPERTY) is True, \
        'use_sim_time parameter was not set to True.'

    # Test that plugin args are passed correctly and ROS args still propagate
    prefix = 'test3_'
    insert_plugin(app, TEST_PLUGIN_NAME, prefix + TEST_PLUGIN_ARGV_PROPERTY,
                  argv=['--args', '--prefix', prefix])

    assert app.property(prefix + TEST_PLUGIN_ARGV_PROPERTY) == ['--prefix', prefix], \
        'The plugin received unexpected arguments: %s' % (
            app.property(prefix + TEST_PLUGIN_ARGV_PROPERTY))
    assert app.property(prefix + TEST_PLUGIN_SIM_TIME_PROPERTY) is True, \
        'use_sim_time parameter was not set to True.'


def _run_in_process(test_cb, argv, config_dir, queue):
    os.environ['XDG_CONFIG_HOME'] = config_dir
    os.environ['QT_QPA_PLATFORM'] = 'offscreen'
    main = MainClass(test_cb, filename=os.path.abspath(__file__))
    ret = main.main(argv=[os.path.abspath(__file__)] + argv)
    queue.put((ret, main.closed_cleanly, main.exc))


@pytest.mark.parametrize('test_cb,argv', [
    (_test_cb_just_opens_and_closes, []),
    (_test_cb_loads_test_plugin, [
        '-s', TEST_PLUGIN_NAME
    ]),
    (_test_cb_sets_ros_parameter, [
        '-s', TEST_PLUGIN_NAME,
        '--ros-args', '-p', 'use_sim_time:=true'
    ]),
    (_test_cb_passes_plugin_args, [
        '-s', TEST_PLUGIN_NAME,
        '--args', '-p', '-q', '--raw'
    ]),
    (_test_cb_passes_plugin_and_ros_args, [
        '-s', TEST_PLUGIN_NAME,
        '--args', '-p', '-q', '--raw',
        '--ros-args', '-p', 'use_sim_time:=true',
    ]),
    (_test_cb_passes_plugin_and_ros_args, [
        '-s', TEST_PLUGIN_NAME,
        '--ros-args', '-p', 'use_sim_time:=true', '--',
        '--args', '-p', '-q', '--raw',
    ]),
    (_test_cb_passes_plugin_and_ros_args, [
        '-s', TEST_PLUGIN_NAME,
        '--ros-args', '-p', 'use_sim_time:=false', '--',
        '--args', '-p', '-q', '--raw',
        '--ros-args', '-p', 'use_sim_time:=true',
    ]),
    (_test_cb_passes_plugin_and_ros_args, [
        '-s', TEST_PLUGIN_NAME,
        '--ros-args', '--params-file',
        os.path.join(os.path.dirname(os.path.abspath(__file__)), 'params.yaml'), '--',
        '--args', '-p', '-q', '--raw',
    ]),
    (_test_cb_runtime_plugins_get_ros_params, [
        '--ros-args', '-p', 'use_sim_time:=true',
    ]),
])
def test_rqt_gui(tmp_path, test_cb, argv):
    ctx = mp.get_context('spawn')
    queue = ctx.Queue()
    proc = ctx.Process(target=_run_in_process, args=(test_cb, argv, str(tmp_path), queue))
    proc.start()
    proc.join(timeout=30)
    assert not proc.is_alive(), 'Test timed out'
    ret, closed_cleanly, exc = queue.get_nowait()
    if exc is not None:
        raise exc
    assert ret == 0
    assert closed_cleanly
