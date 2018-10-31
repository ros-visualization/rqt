# Copyright (c) 2011, Dorian Scholz, TU Darmstadt
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

import rclpy
from rqt_py_common.topic_helpers import get_message_class

class TopicDict(object):

    def __init__(self):
        self.update_topics()

    def get_topics(self):
        return self.topic_dict

    def update_topics(self, node=None):
        # NOTE: This has changed from ROS1 to ROS2 since ROS2 seems to support
        #       multiple msg types on a single topic
        self.topic_dict = {}
        # If no node is passed in then we need to start rclpy and create a node
        # These flags are used to track these changes so that we can restore
        # state on completion
        shutdown_rclpy = False
        destroy_node = False
        if node is None:
            if not rclpy.ok():
                shutdown_rclpy = True
                rclpy.init()

            destroy_node = True
            node = rclpy.create_node("TopicDict__update_topics")
            # Give the node time to learn about the graph
            rclpy.spin_once(node, timeout_sec=0.01)

        topic_names_and_types = node.get_topic_names_and_types()

        # Restore state
        if destroy_node:
            node.destroy_node()
        if shutdown_rclpy:
            rclpy.shutdown()

        for topic_name, topic_types in topic_names_and_types:
            self.topic_dict[topic_name] = []
            for topic_type in topic_types:
                message = get_message_class(topic_type)()
                self.topic_dict[topic_name].append(
                    self._recursive_create_field_dict(topic_type, message))

    def _recursive_create_field_dict(self, slot_name, field):
        field_dict = {}
        field_dict[slot_name] = {
            'type': type(field),
            'children': {},
        }

        if hasattr(field, '__slots__'):
            for child_slot_name in field.__slots__:
                field_dict[slot_name]['children'].update(
                    self._recursive_create_field_dict(
                        child_slot_name, getattr(field, child_slot_name)))
        return field_dict


if __name__ == '__main__':
    rclpy.init()
    import pprint
    pprint.pprint(TopicDict().get_topics())
    rclpy.shutdown()