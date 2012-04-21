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

from __future__ import with_statement, print_function

import sys

import rospkg






                
class Generator:
    
    def __init__(self,
                 dotcode_factory,
                 selected_names = [],
                 excludes = [],
                 depth = 1,
                 with_stacks = False,
                 descendants = True,
                 ancestors = False,
                 hide_transitives = True,
                 interstack_edges = True):
        """
        
        :param hide_transitives: if true, then dependency of children to grandchildren will be hidden if parent has same dependency
        """
        self.dotcode_factory = dotcode_factory
        self.rospack = rospkg.RosPack()
        self.rosstack = rospkg.RosStack()
        self.stacks = {}
        self.packages = {}
        self.edges = []
        self.excludes = excludes
        self.with_stacks = with_stacks
        self.depth = depth
        self.selected_names = selected_names
        self.hide_transitives = hide_transitives
        
        for name in self.selected_names:
            if name is None or name.strip() == '':
                continue
            namefound = False
            if name in self.rospack.list():
                namefound = True
                if descendants:
                    self.add_package_descendants_recursively(name)
                if ancestors:
                    self.add_package_ancestors_recursively(name)
            if name in self.rosstack.list():
                namefound = True
                if descendants:
                    for package_name in self.rosstack.packages_of(name):
                        self.add_package_descendants_recursively(package_name)



    def generate(self):
        graph = self.dotcode_factory.get_graph()
        if self.with_stacks:
            for stackname in self.stacks:
                color = None
                if stackname in self.selected_names:
                    color = 'red'
                g = self.dotcode_factory.add_subgraph_to_graph(graph, stackname, color = color)
                for package_name in self.stacks[stackname]['packages']:
                    color = None
                    if package_name in self.selected_names:
                        color = 'red'
                    self.dotcode_factory.add_node_to_graph(g, package_name, color = color)
        else:
            for package_name in self.packages:
                color = None
                if package_name in self.selected_names:
                    color = 'red'
                self.dotcode_factory.add_node_to_graph(graph, package_name, color = color)
        for edge_tupel in self.edges:
            self.dotcode_factory.add_edge_to_graph(graph, edge_tupel[0], edge_tupel[1])
        return graph
        
    def _add_stack(self, stackname):
        if stackname is None or stackname in self.stacks:
            return
        self.stacks[stackname] = {'packages': []}
        
    def _add_package(self, package_name):
        if package_name in self.packages:
            return False
        self.packages[package_name] = {}
        if self.with_stacks:
            stackname = self.rospack.stack_of(package_name)
            if not stackname is None:
                if not stackname in self.stacks:
                    self._add_stack(stackname)
                self.stacks[stackname]['packages'].append(package_name)
        return True
    
    def _add_edge(self, name1, name2, attributes = None):
        self.edges.append((name1, name2, attributes))

    def add_package_ancestors_recursively(self, package_name, expanded = None, depth = None):
        pass

    def add_package_descendants_recursively(self, package_name, expanded = None, depth = None):
        if package_name in self.excludes:
            return False
        if (depth == 0):
            return False
        if (depth == None):
            depth = self.depth
        self._add_package(package_name)
        if expanded is None:
            expanded = []
        expanded.append(package_name)
        if (depth != 1):
            depends = self.rospack.get_depends(package_name, implicit = False)
            new_nodes = []
            for dep_name in [x for x in depends if x not in self.excludes]:
                if not self.hide_transitives or not dep_name in expanded:
                    new_nodes.append(dep_name)
                    self._add_edge(package_name, dep_name)
                    self._add_package(dep_name)
                    expanded.append(dep_name)
            for dep_name in new_nodes:
                self.add_package_descendants_recursively(package_name = dep_name, 
                                             expanded = expanded,
                                             depth = depth-1)

def generate_dotcode(dotcode_factory,
                     selected_names = [],
                     excludes = [],
                     depth = 3,
                     with_stacks = True,
                     descendants = True,
                     ancestors = False,
                     hide_transitives = True,
                     interstack_edges = True,
                     rank = 'same',
                     ranksep = 0.2,
                     simplify = True,
                     compound = True):
    if depth is None:
        depth = -1
    gen = Generator(dotcode_factory,
                    selected_names = selected_names,
                    excludes = excludes,
                    depth = depth,
                    with_stacks = with_stacks,
                    descendants = descendants,
                    ancestors = ancestors,
                    hide_transitives = hide_transitives,
                    interstack_edges = interstack_edges)

    graph = gen.generate()
    return dotcode_factory.create_dot(graph)

