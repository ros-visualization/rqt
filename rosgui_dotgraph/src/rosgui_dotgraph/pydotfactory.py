# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
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

import pydot
from distutils.version import LooseVersion

# Reference implementation for a dotcode factory

class PydotFactory():
    
    def get_graph(self, graph_type = 'digraph', rank = 'same', simplify = True, ranksep = 0.2, compound = True):
        # Lucid version of pydot bugs with certain settings, not sure which version exactly fixes those
        if LooseVersion(pydot.__version__) > LooseVersion('1.0.10'):
            graph = pydot.Dot('graphname',
                              graph_type = graph_type,
                              rank = rank,
                              simplify = simplify
                              )
            graph.set_ranksep(ranksep);
            graph.set_compound(compound)
        else:
            graph = pydot.Dot('graphname',
                              graph_type = graph_type,
                              rank = rank)
        return graph
    
    def add_node_to_graph(self,
                          graph,
                          nodelabel,
                          shape = 'box',
                          color = None):
        """
        creates a node item for this factory, adds it to the graph.
        Node name can vary from label but must always be same for the same node label
        """
        node = pydot.Node(nodelabel)
        node.set_shape(shape)
        node.set_label(nodelabel)
        
        if color is not None:
            node.set_color('red')
        graph.add_node(node)
        return node
    
    def add_subgraph_to_graph(self,
                              graph,
                              subgraphlabel,
                              rank = 'same',
                              simplify = True,
                              ranksep = 0.2,
                              compound = True,
                              color = None,
                              shape = 'box',
                              style='bold'):
        """
        creates a cluster subgraph  item for this factory, adds it to the graph.
        cluster name can vary from label but must always be same for the same node label.
        Most layouters require cluster names to start with cluster.
        """
        if subgraphlabel is None:
            return None
        g = pydot.Cluster("cluster_%s"%subgraphlabel, rank = rank, simplify = simplify)
        if 'set_style' in g.__dict__:
            g.set_style(style)
        if 'set_shape' in g.__dict__:
            g.set_shape(shape)
        if LooseVersion(pydot.__version__) > LooseVersion('1.0.10'):
            g.set_compound(compound)
            g.set_ranksep(ranksep)
        g.set_label(subgraphlabel)
        if color is not None:
            if 'set_color' in g.__dict__:
                g.set_color('red')
        graph.add_subgraph(g)
        return g

    def add_edge_to_graph(self, graph, nodename1, nodename2):
        edge = pydot.Edge(nodename1, nodename2)
        graph.add_edge(edge)

    def create_dot(self, graph):
        dot = graph.create_dot()
        # sadly pydot generates line wraps cutting between numbers
        return dot.replace("\\\n", "")
