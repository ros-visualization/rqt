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

import roslib
roslib.load_manifest('rosgui_package_graph')

from rosgui.QtBindingHelper import loadUi
from QtCore import QRectF, QPointF
from QtGui import QColor


import edge_item
from edge_item import EdgeItem

import node_item
from node_item import NodeItem

POINTS_PER_INCH = 72

# hack required by pydot
def get_unquoted(item, name):
    value = item.get(name)
    if value is None:
        return None
    try:
        return value.strip('"\n"')
    except AttributeError:
        # not part of the string family
        return value

# approximately, for workarounds (TODO: get this from dotfile somehow)
LABEL_HEIGHT = 30

# Class generating Qt Elements from doctcode
class DotToQtGenerator():

    def getNodeItemForSubgraph(self, subgraph, highlight_level):
        # let pydot imitate pygraphviz api
        attr = {}
        for name in subgraph.get_attributes().iterkeys():
            value = get_unquoted(node, name)
            attr[name] = value
        obj_dic = subgraph.__getattribute__("obj_dict")
        for name in obj_dic:
            if name not in ['nodes', 'attributes', 'parent_graph'] and obj_dic[name] is not None:
                attr[name] = get_unquoted(obj_dic, name)
            elif name == 'nodes':
                for key in obj_dic['nodes']['graph'][0]['attributes']:
                    attr[key] = get_unquoted(obj_dic['nodes']['graph'][0]['attributes'], key)
        subgraph.attr = attr
    
        bb = subgraph.attr['bb'].strip('"').split(',')
        bounding_box = QRectF(0, 0, float(bb[2]) - float(bb[0]), float(bb[3]) -float(bb[1]))
        if 'lp' in subgraph.attr:
            label_pos = subgraph.attr['lp'].strip('"').split(',')
        else:
            label_pos = (float(bb[0]) + (float(bb[2]) - float(bb[0])) / 2, float(bb[1]) + (float(bb[3]) -float(bb[1])) - LABEL_HEIGHT / 2)
        bounding_box.moveCenter(QPointF(float(bb[0]) + (float(bb[2]) - float(bb[0])) / 2, - float(bb[1]) - (float(bb[3]) -float(bb[1]))/2))
        name = subgraph.attr['label']
        color = QColor(subgraph.attr['color']) if 'color' in subgraph.attr else None
        subgraph_nodeitem = NodeItem(highlight_level,
                                     bounding_box,
                                     label = name,
                                     shape = 'box',
                                     color=color,
                                     label_pos=QPointF(float(label_pos[0]), -float(label_pos[1])))
        bounding_box = QRectF(bounding_box)
        # With clusters we have the problem that mouse hovers cannot
        # decide whether to be over the cluster or a subnode. Using
        # just the "title area" solves this. TODO: Maybe using a
        # border region would be even better (multiple RectF)
        bounding_box.setHeight(LABEL_HEIGHT)
        subgraph_nodeitem.set_hovershape(bounding_box)
        return subgraph_nodeitem
    
    def getNodeItemForNode(self, node, highlight_level):
    # let pydot imitate pygraphviz api
        attr = {}
        for name in node.get_attributes().iterkeys():
            value = get_unquoted(node, name)
            attr[name] = value
        obj_dic = node.__getattribute__("obj_dict")
        for name in obj_dic:
            if name not in ['attributes', 'parent_graph'] and obj_dic[name] is not None:
                attr[name] = get_unquoted(obj_dic, name)
        node.attr = attr
        
        # decrease rect by one so that edges do not reach inside
        bounding_box = QRectF(0, 0, POINTS_PER_INCH * float(node.attr['width']) - 1.0, POINTS_PER_INCH * float(node.attr['height']) - 1.0)
        pos = node.attr['pos'].split(',')
        bounding_box.moveCenter(QPointF(float(pos[0]), -float(pos[1])))
        color = QColor(node.attr['color']) if 'color' in node.attr else None
        name = None
        if 'label' in node.attr:
            name = node.attr['label']
        elif 'name' in node.attr:
            name = node.attr['name']
        else:
            print("Error, no label defined for node with attr: %s"%node.attr)
            return
        if name is None:
            # happens on Lucid pygraphviz version
            print("Error, label is None for node %s, pygraphviz version may be too old."%node)
        node_item = NodeItem(highlight_level, bounding_box, name, node.attr.get('shape', 'ellipse'), color)
        #node_item.setToolTip(self._generate_tool_tip(node.attr.get('URL', None)))
        return node_item

    def getEdgeItem(self, edge, nodes, edges, highlight_level):
        # let pydot imitate pygraphviz api
        attr = {}
        for name in edge.get_attributes().iterkeys():
            value = get_unquoted(edge, name)
            attr[name] = value
        edge.attr = attr
    
        label = edge.attr.get('label', None)
        label_pos = edge.attr.get('lp', None)
        label_center = None
        if label_pos is not None:
            label_pos = label_pos.split(',')
            label_center = QPointF(float(label_pos[0]), -float(label_pos[1]))
    
        # try pydot, fallback for pygraphviz
        source_node = edge.get_source() if hasattr(edge, 'get_source') else edge[0]
        destination_node = edge.get_destination() if hasattr(edge, 'get_destination') else edge[1]
    
        # create edge with from-node and to-node
        edge_item = EdgeItem(highlight_level, edge.attr['pos'], label_center, label, nodes[source_node], nodes[destination_node])

        if label is None:
            # for sibling detection
            label = "%s_%s"%(source_node, destination_node)
        
        # symmetrically add all sibling edges with same label
        if label not in edges:
            edges[label] = []
        for sibling in edges[label]:
            edge_item.add_sibling_edge(sibling)
            sibling.add_sibling_edge(edge_item)
        edges[label].append(edge_item)
        return edge_item

    def dotcode_to_qt_items(self, dotcode, highlight_level):
        """
        takes dotcode, runs layout, and creates qt items based on the dot layout.
        returns two dicts, one mapping node names to Node_Item, one mapping edge names to lists of Edge_Item
        """
        # layout graph
        if dotcode is None:
            return {}, {}
        graph = pydot.graph_from_dot_data(dotcode.encode("ascii","ignore"))

        #graph = pygraphviz.AGraph(string=self._current_dotcode, strict=False, directed=True)
        #graph.layout(prog='dot')
        
        # let pydot imitate pygraphviz api
        graph.nodes_iter = graph.get_node_list
        graph.edges_iter = graph.get_edge_list
        graph.subgraphs_iter = graph.get_subgraph_list

        nodes = {}
        for subgraph in graph.subgraphs_iter():
            subgraph_nodeitem = self.getNodeItemForSubgraph(subgraph, highlight_level)

            subgraph.nodes_iter = subgraph.get_node_list
            nodes[subgraph.get_name()] = subgraph_nodeitem
            for node in subgraph.nodes_iter():
                # hack required by pydot
                if node.get_name() in ('graph', 'node', 'empty'):
                    continue
                nodes[node.get_name()] = self.getNodeItemForNode(node, highlight_level)
        for node in graph.nodes_iter():
            # hack required by pydot
            if node.get_name() in ('graph', 'node', 'empty'):
                continue
            nodes[node.get_name()] = self.getNodeItemForNode(node, highlight_level)

        edges = {} # is irrelevant?
        for subgraph in graph.subgraphs_iter():
            subgraph.edges_iter = subgraph.get_edge_list
            for edge in subgraph.edges_iter():
                edge_item = self.getEdgeItem(edge, nodes, edges, highlight_level)
        for edge in graph.edges_iter():
            edge_item = self.getEdgeItem(edge, nodes, edges, highlight_level)

        return nodes, edges
