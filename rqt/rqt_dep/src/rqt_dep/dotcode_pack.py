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

import re
import roslib

from rospkg.common import ResourceNotFound
from qt_dotgraph.colors import get_color_for_string

MAX_EDGES = 1500


def matches_any(name, patternlist):
    for pattern in patternlist:
        if name == pattern:
            return True
        if re.match("^[a-zA-Z0-9_]+$", pattern) is None:
            if re.match(pattern, name) is not None:
                return True
    return False


class RosPackageGraphDotcodeGenerator:

    def __init__(self, rospack, rosstack):
        """
        :param rospack: use rospkg.RosPack()
        :param rosstack: use rospkg.RosStack()
        """
        self.rospack = rospack
        self.rosstack = rosstack
        self.stacks = {}
        self.packages = {}
        self.edges = []
        self.last_drawargs = None
        self.last_selection = None

    def generate_dotcode(self,
                         dotcode_factory,
                         selected_names=[],
                         excludes=[],
                         depth=3,
                         with_stacks=True,
                         descendants=True,
                         ancestors=True,
                         hide_transitives=True,
                         mark_selected=True,
                         colortheme=None,
                         rank='same',  # None, same, min, max, source, sink
                         ranksep=0.2,  # vertical distance between layers
                         rankdir='TB',  # direction of layout (TB top > bottom, LR left > right)
                         simplify=True,  # remove double edges
                         force_refresh=False):
        """

        :param hide_transitives: if true, then dependency of children to grandchildren will be hidden if parent has same dependency
        """

        # defaults
        selected_names = filter(lambda x: x is not None and x != '', selected_names)
        excludes = filter(lambda x: x is not None and x != '', excludes)
        if selected_names is None or selected_names == []:
            selected_names = ['.*']
            self.depth = 1
        if depth is None:
            depth = -1

        # update arguments

        selection_args = {
            "dotcode_factory" : dotcode_factory,
            "with_stacks" : with_stacks,
            "depth" : depth,
            "hide_transitives" : hide_transitives,
            "selected_names" : selected_names,
            "excludes" : excludes,
            "ancestors" : ancestors,
            "descendants" : descendants
            }

        # if selection did not change, we need not build up the graph again
        selection_changed = False
        if self.last_selection != selection_args:
            selection_changed = True
            self.last_selection = selection_args

            self.dotcode_factory = dotcode_factory
            self.with_stacks = with_stacks
            self.depth = depth
            self.hide_transitives = hide_transitives
            self.selected_names = selected_names
            self.excludes = excludes
            self.ancestors = ancestors
            self.descendants = descendants

        if force_refresh or selection_changed:
            self.stacks = {}
            self.packages = {}
            self.edges = []
            # update internal graph structure
            for name in self.rospack.list():
                if matches_any(name, self.selected_names):
                    if descendants:
                        self.add_package_descendants_recursively(name)
                    if ancestors:
                        self.add_package_ancestors_recursively(name)
            for stackname in self.rosstack.list():
                if matches_any(stackname, self.selected_names):
                    for package_name in self.rosstack.packages_of(stackname):
                        if descendants:
                            self.add_package_descendants_recursively(package_name)
                        if ancestors:
                            self.add_package_ancestors_recursively(package_name)

        drawing_args = {
            'dotcode_factory': dotcode_factory,
            "rank" : rank,
            "rankdir" : rankdir,
            "ranksep" : ranksep,
            "simplify" : simplify,
            "colortheme": colortheme,
            "mark_selected" : mark_selected
            }

        # if selection and display args did not change, no need to generate dotcode
        display_changed = False
        if self.last_drawargs != drawing_args:
            display_changed = True
            self.last_drawargs = drawing_args

            self.dotcode_factory = dotcode_factory
            self.rank = rank
            self.rankdir = rankdir
            self.ranksep = ranksep
            self.simplify = simplify
            self.colortheme = colortheme
            self.dotcode_factory = dotcode_factory
            self.mark_selected = mark_selected

        #generate new dotcode
        if force_refresh or selection_changed or display_changed:
            self.graph = self.generate(self.dotcode_factory)
            self.dotcode = dotcode_factory.create_dot(self.graph)

        return self.dotcode

    def generate(self, dotcode_factory):
        graph = dotcode_factory.get_graph(rank=self.rank,
                                          rankdir=self.rankdir,
                                          ranksep=self.ranksep,
                                          simplify=self.simplify)
        # print("In generate", self.with_stacks, len(self.stacks), len(self.packages), len(self.edges))
        packages_in_stacks = []
        if self.with_stacks:
            for stackname in self.stacks:
                color = None
                if self.mark_selected and not '.*' in self.selected_names and matches_any(stackname, self.selected_names):
                    color = 'red'
                else:
                    if self.colortheme is not None:
                        color = get_color_for_string(stackname)
                g = dotcode_factory.add_subgraph_to_graph(graph,
                                                          stackname,
                                                          color=color,
                                                          rank=self.rank,
                                                          rankdir=self.rankdir,
                                                          ranksep=self.ranksep,
                                                          simplify=self.simplify)

                for package_name in self.stacks[stackname]['packages']:
                    packages_in_stacks.append(package_name)
                    self._generate_package(dotcode_factory, g, package_name)

        for package_name in self.packages:
            if package_name not in packages_in_stacks:
                self._generate_package(dotcode_factory, graph, package_name)
        if (len(self.edges) < MAX_EDGES):
            for edge_tupel in self.edges:
                dotcode_factory.add_edge_to_graph(graph, edge_tupel[0], edge_tupel[1])
        else:
            print("Too many edges %s > %s, abandoning generation of edge display" % (len(self.edges), MAX_EDGES))
        return graph

    def _generate_package(self, dotcode_factory, graph, package_name):
        color = None
        if self.mark_selected and not '.*' in self.selected_names and matches_any(package_name, self.selected_names):
            color = 'red'
        dotcode_factory.add_node_to_graph(graph, package_name, color=color)

    def _add_stack(self, stackname):
        if stackname is None or stackname in self.stacks:
            return
        self.stacks[stackname] = {'packages': []}

    def _add_package(self, package_name, parent=None):
        """
        adds object based on package_name to self.packages
        :param parent: packagename which referenced package_name (for debugging only)
        """
        if package_name in self.packages:
            return False
        self.packages[package_name] = {}

        if self.with_stacks:
            try:
                stackname = self.rospack.stack_of(package_name)
            except ResourceNotFound as e:
                print('RosPackageGraphDotcodeGenerator._add_package(%s), parent %s: ResourceNotFound:' % (package_name, parent), e)
                stackname = None
            if not stackname is None and stackname != '':
                if not stackname in self.stacks:
                    self._add_stack(stackname)
                self.stacks[stackname]['packages'].append(package_name)
        return True

    def _add_edge(self, name1, name2, attributes=None):
        self.edges.append((name1, name2, attributes))

    def add_package_ancestors_recursively(self, package_name, expanded_up=None, depth=None, implicit=False, parent=None):
        """
        :param package_name: the name of package for which to add ancestors
        :param expanded_up: names that have already been expanded (to avoid cycles)
        :param depth: how many layers to follow
        :param implicit: arg to rospack
        :param parent: package that referenced package_name for error message only
        """
        if matches_any(package_name, self.excludes):
            return False
        if (depth == 0):
            return False
        if (depth == None):
            depth = self.depth
        self._add_package(package_name, parent=parent)
        if expanded_up is None:
            expanded_up = []
        expanded_up.append(package_name)
        if (depth != 1):
            try:
                depends_on = self.rospack.get_depends_on(package_name, implicit=implicit)
            except ResourceNotFound as e:
                print('RosPackageGraphDotcodeGenerator.add_package_ancestors_recursively(%s), parent %s: ResourceNotFound:' % (package_name, parent), e)
                depends_on = []
            new_nodes = []
            for dep_on_name in [x for x in depends_on if not matches_any(x, self.excludes)]:
                if not self.hide_transitives or not dep_on_name in expanded_up:
                    new_nodes.append(dep_on_name)
                    self._add_edge(dep_on_name, package_name)
                    self._add_package(dep_on_name, parent=package_name)
                    expanded_up.append(dep_on_name)
            for dep_on_name in new_nodes:
                self.add_package_ancestors_recursively(package_name=dep_on_name,
                                                       expanded_up=expanded_up,
                                                       depth=depth - 1,
                                                       implicit=implicit,
                                                       parent=package_name)

    def add_package_descendants_recursively(self, package_name, expanded=None, depth=None, implicit=False, parent=None):
        if matches_any(package_name, self.excludes):
            return False
        if (depth == 0):
            return False
        if (depth == None):
            depth = self.depth
        self._add_package(package_name, parent=parent)
        if expanded is None:
            expanded = []
        expanded.append(package_name)
        if (depth != 1):
            try:
                depends = self.rospack.get_depends(package_name, implicit=implicit)
            except ResourceNotFound as e:
                print('RosPackageGraphDotcodeGenerator.add_package_descendants_recursively(%s), parent: %s: ResourceNotFound:' % (package_name, parent), e)
                depends = []
            new_nodes = []
            for dep_name in [x for x in depends if not matches_any(x, self.excludes)]:
                if not self.hide_transitives or not dep_name in expanded:
                    new_nodes.append(dep_name)
                    self._add_edge(package_name, dep_name)
                    self._add_package(dep_name, parent=package_name)
                    expanded.append(dep_name)
            for dep_name in new_nodes:
                self.add_package_descendants_recursively(package_name=dep_name,
                                                         expanded=expanded,
                                                         depth=depth - 1,
                                                         implicit=implicit,
                                                         parent=package_name)


