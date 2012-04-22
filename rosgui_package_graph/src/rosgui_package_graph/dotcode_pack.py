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

import re

MAX_EDGES=1500

def matches_any(name, patternlist):
    for pattern in patternlist:
        if name == pattern:
            return True
        if re.match("^[a-zA-Z0-9_]+$", pattern) is None:
            if re.match(pattern, name) is not None:
                return True
    return False
    
class RosPackageGraphDotcodeGenerator:
    
    def __init__(self):
        self.rospack = rospkg.RosPack()
        self.rosstack = rospkg.RosStack()
        self.stacks = {}
        self.packages = {}
        self.edges = []
        self.selected_names = None
        self.excludes = None
        self.depth = None
        self.hide_transitives = None
        self.with_stacks = None
        self.last_graph = None
        self.graph = None
        self.dotcode = None
        self.rank = None
        self.ranksep = None
        self.simplify = None
        self.compound = None
        
        
    def generate_dotcode(self,
                         dotcode_factory,
                         selected_names = [],
                         excludes = [],
                         depth = 3,
                         with_stacks = True,
                         descendants = True,
                         ancestors = False,
                         hide_transitives = True,
                         interstack_edges = True,
                         rank = 'source', # None, same, min, max, source, sink
                         ranksep = 0.2, # vertical distance between layers
                         simplify = True, # remove double edges
                         compound = True, # allow edges bewteen subgraphs
                         force_refresh = False):
        """
        
        :param hide_transitives: if true, then dependency of children to grandchildren will be hidden if parent has same dependency
        """

        # update arguments
        self.dotcode_factory = dotcode_factory

        

        selected_names = filter(lambda x: x is not None and x != '', selected_names)
        excludes = filter(lambda x: x is not None and x != '', excludes)
        if selected_names is None or selected_names == []:
            selected_names = ['.*']
            self.depth = 1
            
        selection_changed = False
        
        if self.with_stacks != with_stacks:
            selection_changed = True
            self.with_stacks = with_stacks
            
        if depth is None:
            depth = -1
        if self.depth != depth:
            selection_changed = True
            self.depth = depth

        if self.hide_transitives != hide_transitives:
            selection_changed = True
            self.hide_transitives = hide_transitives
            print(self.hide_transitives)

        if self.selected_names != selected_names:
            selection_changed = True
            self.selected_names = selected_names
            
        if self.excludes != excludes:
            selection_changed = True
            self.excludes = excludes

        if force_refresh or selection_changed:
            self.stacks = {}
            self.packages = {}
            self.edges = []
            # update internal graph structure
            for name in self.rospack.list():
                if matches_any(name, self.selected_names):
                    namefound = True
                    if descendants:
                        self.add_package_descendants_recursively(name)
                    if ancestors:
                        self.add_package_ancestors_recursively(name)
            for stackname in self.rosstack.list():
                if matches_any(stackname, self.selected_names):
                    namefound = True
                    for package_name in self.rosstack.packages_of(stackname):
                        if descendants:
                             self.add_package_descendants_recursively(package_name)
                        if ancestors:
                            self.add_package_ancestors_recursively(package_name)

        display_changed = False
        if self.rank != rank:
            display_changed = True
            self.rank = rank
        if self.ranksep != ranksep:
            display_changed = True
            self.ranksep = ranksep
        if self.simplify != simplify:
            display_changed = True
            self.simplify = simplify
        if self.compound != compound:
            display_changed = True
            self.compound = compound
        if self.dotcode_factory != dotcode_factory:
            display_changed = True
            self.dotcode_factory = dotcode_factory

                            
        graph_changed = False
        #generate new dotcode
        if force_refresh or selection_changed or display_changed:
            graph_changed = True
            self.graph = self.generate(self.dotcode_factory)
        if graph_changed:
            self.dotcode = dotcode_factory.create_dot(self.graph)
        return self.dotcode

    def generate(self, dotcode_factory):
        graph = dotcode_factory.get_graph(rank = self.rank,
                                          ranksep = self.ranksep,
                                          simplify = self.simplify,
                                          compound = self.compound)
        # print("In generate", self.with_stacks, len(self.stacks), len(self.packages), len(self.edges))
        if self.with_stacks:
            for stackname in self.stacks:
                color = None
                if not '.*' in self.selected_names and matches_any(stackname, self.selected_names):
                    color = 'red'
                g = dotcode_factory.add_subgraph_to_graph(graph,
                                                          stackname,
                                                          color = color,
                                                          rank = self.rank,
                                                          ranksep = self.ranksep,
                                                          simplify = self.simplify,
                                                          compound = self.compound)
                for package_name in self.stacks[stackname]['packages']:
                    color = None
                    if not '.*' in self.selected_names and matches_any(package_name, self.selected_names):
                        color = 'red'
                    dotcode_factory.add_node_to_graph(g, package_name, color = color)
        else:
            for package_name in self.packages:
                color = None
                if not '.*' in self.selected_names and matches_any(package_name, self.selected_names):
                    color = 'red'
                dotcode_factory.add_node_to_graph(graph, package_name, color = color)
        if (len(self.edges) < MAX_EDGES):
            for edge_tupel in self.edges:
                dotcode_factory.add_edge_to_graph(graph, edge_tupel[0], edge_tupel[1])
        else:
            print("Too many edges %s > %s, abandoning generation of edge display"%(len(self.edges), MAX_EDGES))
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
        if matches_any(package_name, self.excludes):
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
            for dep_name in [x for x in depends if not matches_any(x, self.excludes)]:
                if not self.hide_transitives or not dep_name in expanded:
                    new_nodes.append(dep_name)
                    self._add_edge(package_name, dep_name)
                    self._add_package(dep_name)
                    expanded.append(dep_name)
            for dep_name in new_nodes:
                self.add_package_descendants_recursively(package_name = dep_name, 
                                             expanded = expanded,
                                             depth = depth-1)

    
