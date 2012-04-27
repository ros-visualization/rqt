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

# this is a modified version of rx/rxgraph/src/rxgraph/dotcode.py

import rosgraph.impl.graph

# node/node connectivity
NODE_NODE_GRAPH = 'node_node'
# node/topic connections where an actual network connection exists
NODE_TOPIC_GRAPH = 'node_topic'
# all node/topic connections, even if no actual network connection
NODE_TOPIC_ALL_GRAPH = 'node_topic_all'

import urllib

QUIET_NAMES = ['/diag_agg', '/runtime_logger', '/pr2_dashboard', '/rviz', '/rosout', '/cpu_monitor', '/monitor', '/hd_monitor', '/rxloggerlevel', '/clock']

class RosGraphDotcodeGenerator:

    def __init__(self):
        self.dotcode_factory = None

    def _add_edge(self, edge, dotgraph, is_topic=False):
        if is_topic:
            self.dotcode_factory.add_edge_to_graph(dotgraph, edge.start, edge.end, label = edge.label, url = 'topic:%s' % edge.label)
        else:
            self.dotcode_factory.add_edge_to_graph(dotgraph, edge.start, edge.end, label = edge.label)
            
    def _add_node(self, node, rosgraphinst, dotgraph, quiet):
        if node in rosgraphinst.bad_nodes:
            if quiet:
                return ''
            bn = rosgraphinst.bad_nodes[node]
            if bn.type == rosgraph.impl.graph.BadNode.DEAD:
                self.dotcode_factory.add_node_to_graph(dotgraph,
                                                       nodelabel = node,
                                                       shape = "doublecircle",
                                                       url=node,
                                                       color = "red")
            else:
                self.dotcode_factory.add_node_to_graph(dotgraph,
                                                       nodelabel = node,
                                                       shape = "doublecircle",
                                                       url=node,
                                                       color = "orange")
        else:
            self.dotcode_factory.add_node_to_graph(dotgraph,
                                                   nodelabel = node,
                                                   shape = 'box',
                                                   url=node)
            
    def _add_topic_node(self, node, dotgraph, quiet):
        label = rosgraph.impl.graph.node_topic(node)
        self.dotcode_factory.add_node_to_graph(dotgraph,
                                               nodelabel = "%s"%label,
                                               shape = 'box',
                                               url="topic:%s"%label)
    

    def _quiet_filter(self, name):
        # ignore viewers
        for n in QUIET_NAMES:
            if n in name:
                return False
        return True
    
    def _quiet_filter_edge(self, edge):
        for quiet_label in ['/time', '/clock', '/rosout']:
            if quiet_label == edge.label:
                return False
        return self._quiet_filter(edge.start) and self._quiet_filter(edge.end)
    
    def generate_namespaces(self, rosgraph, graph_mode, quiet=False):
        """
        Determine the namespaces of the nodes being displayed
        """
        namespaces = []
        if graph_mode == NODE_NODE_GRAPH:
            nodes = rosgraph.nn_nodes
            if quiet:
                nodes = [n for n in nodes if not n in QUIET_NAMES]
            namespaces = list(set([roslib.names.namespace(n) for n in nodes]))
    
        elif graph_mode == NODE_TOPIC_GRAPH or \
                 graph_mode == NODE_TOPIC_ALL_GRAPH:
            nn_nodes = rosgraph.nn_nodes
            nt_nodes = rosgraph.nt_nodes
            if quiet:
                nn_nodes = [n for n in nn_nodes if not n in QUIET_NAMES]
                nt_nodes = [n for n in nt_nodes if not n in QUIET_NAMES]
            if nn_nodes or nt_nodes:
                namespaces = [roslib.names.namespace(n) for n in nn_nodes]
            # an annoyance with the rosgraph library is that it
            # prepends a space to topic names as they have to have
            # different graph node namees from nodes. we have to strip here
            namespaces.extend([roslib.names.namespace(n[1:]) for n in nt_nodes])

        return list(set(namespaces))
    
    def _filter_edges(self, edges, nodes):
        # currently using and rule as the or rule generates orphan nodes with the current logic
        return [e for e in edges if e.start in nodes and e.end in nodes]
    
    
    def generate_dotcode(self,
                         rosgraphinst,
                         ns_filter,
                         graph_mode,
                         dotcode_factory,
                         orientation = 'LR',
                         rank = 'same',   # None, same, min, max, source, sink
                         ranksep = 0.2,   # vertical distance between layers
                         rankdir = 'TB',  # direction of layout (TB top > bottom, LR left > right)
                         simplify = True, # remove double edges
                         quiet=False):
        """
        @param rosgraphinst: RosGraph instance
        @param ns_filter: namespace filter (must be canonicalized with trailing '/')
        @type  ns_filter: string
        @param graph_mode str: NODE_NODE_GRAPH | NODE_TOPIC_GRAPH | NODE_TOPIC_ALL_GRAPH
        @type  graph_mode: str
        @param orientation: rankdir value (see ORIENTATIONS dict)
        @type  dotcode_factory: object
        @param dotcode_factory: abstract factory manipulating dot language objects
        @return: dotcode generated from graph singleton
        @rtype: str
        """
        
        self.dotcode_factory = dotcode_factory
        #print "generate_dotcode", graph_mode
        if ns_filter:
            name_filter = ns_filter[:-1]

        # result = "digraph G {\n  rankdir=%(orientation)s;\n%(nodes_str)s\n%(edges_str)s}\n" % vars()
        dotgraph = dotcode_factory.get_graph(rank = rank,
                                          ranksep = ranksep,
                                          simplify = simplify,
                                          rankdir = orientation)
            
        # create the node definitions
        if graph_mode == NODE_NODE_GRAPH:
            nodes = rosgraphinst.nn_nodes
            if quiet:
                nodes = [n for n in nodes if not n in QUIET_NAMES]
            if ns_filter and ns_filter != '/':
                nodes = [n for n in nodes if n.startswith(ns_filter) or n == name_filter]

            for n in nodes:
                self._add_node(n, rosgraphinst=rosgraphinst, dotgraph=dotgraph, quiet=quiet)
                
    
        elif graph_mode == NODE_TOPIC_GRAPH or \
                 graph_mode == NODE_TOPIC_ALL_GRAPH:
            nn_nodes = rosgraphinst.nn_nodes
            nt_nodes = rosgraphinst.nt_nodes
            if quiet:
                nn_nodes = [n for n in nn_nodes if not n in QUIET_NAMES]
                nt_nodes = [n for n in nt_nodes if not n in QUIET_NAMES]
            if ns_filter and ns_filter != '/':
                nn_nodes = [n for n in nn_nodes if n.startswith(ns_filter) or n == name_filter]
                nt_nodes = [n for n in nt_nodes if n[1:].startswith(ns_filter) or n[1:] == name_filter]
    
            if nn_nodes or nt_nodes:
                for n in nn_nodes:
                    self._add_node(n, rosgraphinst=rosgraphinst, dotgraph=dotgraph, quiet=quiet)
                for n in nt_nodes:
                    self._add_topic_node(n, dotgraph=dotgraph, quiet=quiet)
            nodes = list(nn_nodes) + list(nt_nodes)
    
        # create the edge definitions
        if graph_mode == NODE_NODE_GRAPH:
            edges = rosgraphinst.nn_edges
        elif graph_mode == NODE_TOPIC_GRAPH:
            edges = rosgraphinst.nt_edges
        else:
            edges = rosgraphinst.nt_all_edges
            
        if quiet:
            edges = filter(self._quiet_filter_edge, edges)
      
        edges = self._filter_edges(edges, nodes)
        
        for e in edges:
            self._add_edge(e, dotgraph=dotgraph, is_topic=(graph_mode == NODE_NODE_GRAPH))


        self.dotcode = dotcode_factory.create_dot(dotgraph)

        return self.dotcode
        
      
