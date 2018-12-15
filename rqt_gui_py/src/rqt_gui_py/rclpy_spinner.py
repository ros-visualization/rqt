# Copyright 2018, PickNik Consulting
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from queue import Queue

from python_qt_binding.QtCore import qDebug, QThread, qWarning
import rclpy
from rclpy.client import Client
from rclpy.executors import MultiThreadedExecutor
from rclpy.subscription import Subscription


class RclpySpinner(QThread):

    def __init__(self, node):
        super().__init__()
        self._node = node
        self._abort = False
        self._listener_queue = Queue()

    def run(self):
        qDebug('Start called on RclpySpinner, spinning ros2 node')
        executor = MultiThreadedExecutor()
        executor.add_node(self._node)
        while rclpy.ok() and not self._abort:
            executor.spin_once(timeout_sec=1.0)

            # Remove subscribers and clients that have been registered for destruction
            while not self._listener_queue.empty():
                listener = self._listener_queue.get()
                self.__destroy_listener__(listener)

        if not self._abort:
            qWarning('rclpy.shutdown() was called before QThread.quit()')

    def quit(self):  # noqa: A003
        qDebug('Quit called on RclpySpinner')
        self._abort = True
        super().quit()

    def register_listeners_for_destruction(self, *listeners):
        """
        Add subscriptions or service clients to the spinner's queue for destruction.

        Subscription and clients cannot be destroyed while the executor is spinning, these will be
        added to the queue and destroyed after spin_once has completed.

        :param *listeners: The listeners that are to be destroyed
        """
        for listener in listeners:
            if not isinstance(listener, Subscription) and not isinstance(listener, Client):
                qWarning('Invalid object of type "{}" called for listener destruction'.format(
                         type(listener)))
                continue
            qDebug('Registering destruction for listener of type "{}"'.format(type(listener)))
            self._listener_queue.put(listener)

    def __destroy_listener__(self, listener):
        """
        Destroy a subscription or a service.

        This is not thread-safe, do not call this function from outside of this class, use
        RclpySpinner.register_listeners_for_destruction() instead.
        """
        if isinstance(listener, Subscription):
            self._node.destroy_subscription(listener)
        elif isinstance(listener, Client):
            self._node.destroy_client(listener)
        else:
            qWarning('RclpySpinner.__destroy_listener__() unknown listener of type "{}"'.format(
                     type(listener)))
