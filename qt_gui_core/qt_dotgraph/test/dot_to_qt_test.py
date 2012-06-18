#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
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

import unittest

from qt_dotgraph.dot_to_qt import DotToQtGenerator, get_unquoted

class DotToQtGeneratorTest(unittest.TestCase):

    @unittest.skip("fails with Segfault in Nodeitem.__init__()")
    def test_simpleIntegration(self):
        gen = DotToQtGenerator()
        dotcode = 'digraph graphname {\n\tgraph [rank=same];\n\tnode [label="\\N"];\n\tgraph [bb="0,0,56,116"];\n\tsubgraph cluster_foo {\n\t\tgraph [label=foo,\n\t\t\tbb="1,1,100,101"];\n\t}\n\tfoo [label=foo, shape=box, pos="28,98", width="0.75", height="0.50"];\n\tedge_ [label=edge_, shape=box, pos="28,26", width="0.78", height="0.50"];\n\tfoo -> edge_ [pos="e,28,44 28,80 28,72 28,63 28,54"];\n}\n'
        (nodes, edges) = gen.dotcode_to_qt_items(dotcode, 1)
        self.assertEqual(3, len(nodes)) # also for stack
        self.assertEqual(1, len(nodes))

    def test_unquoted(self):
        self.assertEqual("foo", get_unquoted({'bar': 'foo'}, 'bar'))
