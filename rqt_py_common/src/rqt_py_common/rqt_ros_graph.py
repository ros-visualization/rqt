# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Isaac Saito

from python_qt_binding.QtCore import Qt


class RqtRosGraph(object):
    DELIM_GRN = '/'

    @staticmethod
    def get_lower_grn_dfs(model_index, grn_prev=''):
        """
        Traverse all children treenodes and returns a list of "partial" GRNs.

        Partial means that this method returns names under current level.

        Ex. Consider a tree like this:

        Root
         |--TopitemA
         |    |--1
         |      |--2
         |        |--3
         |          |--4
         |          |--5
         |            |--6
         |            |--7
         |--TopitemB

        Re-formatted in GRN (omitting root):

          TopitemA/1/2/3/4
          TopitemA/1/2/3/5/6
          TopitemA/1/2/3/5/7
          TopitemB

         Might not be obvious from tree representation but there are 4 nodes as
         GRN form suggests.

         (doc from here TBD)

        :type model_index: QModelIndex
        :type grn_prev: str
        :rtype: str[]
        """
        i_child = 0
        list_grn_children_all = []
        while True:  # Loop per child.
            grn_curr = grn_prev + RqtRosGraph.DELIM_GRN + \
                str(model_index.data())
            child_qmindex = model_index.model().index(i_child, 0, model_index)

            if (not child_qmindex.isValid()):
                if i_child == 0:
                    # Only when the current node has no children, add current
                    # GRN to the returning list.
                    list_grn_children_all.append(grn_curr)
                return list_grn_children_all

            list_grn_children = RqtRosGraph.get_lower_grn_dfs(child_qmindex,
                                                              grn_curr)
            for child_grn in list_grn_children:
                child_grn = (grn_prev +
                             (RqtRosGraph.DELIM_GRN + grn_curr) +
                             (RqtRosGraph.DELIM_GRN + child_grn))

            list_grn_children_all = list_grn_children_all + list_grn_children
            i_child += 1
        return list_grn_children_all

    @staticmethod
    def get_upper_grn(model_index, str_grn):
        if model_index.data(Qt.ItemDataRole.DisplayRole) is None:
            return str_grn
        str_grn = (RqtRosGraph.DELIM_GRN +
                   str(model_index.data(Qt.ItemDataRole.DisplayRole)) +
                   str_grn)
        return RqtRosGraph.get_upper_grn(model_index.parent(), str_grn)
