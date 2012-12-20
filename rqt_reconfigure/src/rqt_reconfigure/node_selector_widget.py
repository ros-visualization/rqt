# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
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
#
# Author: Isaac Saito

from __future__ import division

import os
import sys

import dynamic_reconfigure as dyn_reconf
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, Signal
from python_qt_binding.QtGui import QStandardItemModel, QWidget
import rospkg
import rospy
import rosservice

from .parameter_item import ParameterItem

class NodeSelectorWidget(QWidget):
    _COL_NAMES = ['Node']

    # public signal
    sig_node_selected = Signal(str)

    def __init__(self):
        super(NodeSelectorWidget, self).__init__()
        self.stretch = None

        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_reconfigure'), 'resource', 'node_selector.ui')
        loadUi(ui_file, self)

        #  Setup treeview and models
        self._std_model = QStandardItemModel()
        self._node_selector_view.setModel(self._std_model)
        self._rootitem = self._std_model.invisibleRootItem()

        self._nodes_previous = None

        # Calling this method updates the list of the node. 
        # Initially done only once.
        self._update_nodetree()
        
        # Setting slot for when user clickes on QTreeView.
        selectionModel = self._node_selector_view.selectionModel()
        selectionModel.selectionChanged.connect(self._selection_changed_slot)

        #TODO(Isaac): Needs auto-update function enabled, once another function
        #             that updates node tree with maintaining collapse/expansion
        #             state. http://goo.gl/GuwYp can be a help.
        
#        self.timer = QTimer()
#        self.timer.timeout.connect(self._refresh_nodes)
#        self.timer.start(5000) #  5sec interval is fast enough.

        self._collapse_button.pressed.connect(self._node_selector_view.collapseAll)
        self._expand_button.pressed.connect(self._node_selector_view.expandAll)

    def _selection_changed_slot(self, selected, deselected):
        """
        Overriden from QItemSelectionModel.
        
        :type new_item_select: QItemSelection
        :type old_item_select: QItemSelection
        """

        index_current = self._node_selector_view.selectionModel().currentIndex()
        rospy.logdebug('_selection_changed_slot row=%d col=%d data=%s ' +
                       'data.parent=%s child(0, 0)=%s',
                       index_current.row(),
                       index_current.column(),
                       index_current.data(Qt.DisplayRole),
                       index_current.parent().data(Qt.DisplayRole),
                       index_current.child(0, 0).data(Qt.DisplayRole))

        if not index_current.child(0, 0).data(Qt.DisplayRole) == None:
            return #  Meaning the tree has no nodes.

        #  get the text of the selected item
        node_name_selected = self.get_full_grn_recur(index_current, '')
        rospy.logdebug('_selection_changed_slot node_name_selected=%s',
                       node_name_selected)
        self.sig_node_selected.emit(node_name_selected)

    def get_full_grn_recur(self, model_index, str_grn):
        """
        
        Create full path format of GRN (Graph Resource Names, see  
        http://www.ros.org/wiki/Names). 
        
        An example: /wide_stereo/left/image_color/compressed
        
        Bulid GRN by recursively transcending parents & children of 
        QModelIndex instance.
        
        Upon its very 1st call, the argument is the index where user clicks on on
        the view object (here QTreeView is used but should work with other View).
        str_grn can be 0-length string.           
        
        :type model_index: QModelIndex
        :type str_grn: str
        :param str_grn: This could be an incomplete or a complete GRN format.         
        """
        #TODO(Isaac) Consider moving this to rqt_py_common. 

        rospy.logdebug('get_full_grn_recur in str=%s', str_grn)
        if model_index.data(Qt.DisplayRole) == None:
            return str_grn
        str_grn = '/' + str(model_index.data(Qt.DisplayRole)) + str_grn
        rospy.logdebug('get_full_grn_recur out str=%s', str_grn)
        return self.get_full_grn_recur(model_index.parent(), str_grn)

    def _update_nodetree(self):
        """
        """

        #TODO(Isaac): 11/25/2012 dynamic_reconfigure only returns params that 
        #             are associated with nodes. In order to handle independent
        #             params, different approach needs taken.
        try:
            nodes = dyn_reconf.find_reconfigure_services()
        except rosservice.ROSServiceIOException as e:
            rospy.logerr("Reconfigure GUI cannot connect to master.")
            raise e  #TODO Make sure 'raise' here returns or finalizes this func.

        if not nodes == self._nodes_previous:
            paramname_prev = ''
            paramitem_top_prev = None
            i_debug = 0
            for node_name_grn in nodes:
                p = ParameterItem(node_name_grn)
                p.set_param_name(node_name_grn)
                names = p.get_param_names()

                i_debug += 1
                rospy.logdebug('_update_nodetree i=%d names=%s',
                               i_debug, names)

                self._add_tree_node(p, self._rootitem, names)

    def _add_tree_node(self, param_item_full, stditem_parent, child_names_left):
        """
        
        Evaluate current node and the previous node on the same depth.
        If the name of both nodes is the same, current node instance is ignored.
        If not, the current node gets added to the same parent node.
        At the end, this function gets called recursively going down 1 level.
        
        :type param_item_full: ParameterItem
        :type stditem_parent: QStandardItem.
        :type child_names_left: List of str
        :param child_names_left: List of strings that is sorted in hierarchical 
                                 order of params.
        """
        #TODO(Isaac): Consider moving to rqt_py_common. 

        name_curr = child_names_left.pop(0)
        stditem_curr = ParameterItem(param_item_full.get_raw_param_name())

        # item at the bottom is your most recent node.
        row_index_parent = stditem_parent.rowCount() - 1

        # Obtain and instantiate prev node in the same depth.
        name_prev = ''
        stditem_prev = None
        if not stditem_parent.child(row_index_parent) == None:
            stditem_prev = stditem_parent.child(row_index_parent)
            name_prev = stditem_prev.text()

        stditem = None
        if name_prev != name_curr:
            stditem_curr.setText(name_curr)
            stditem_parent.appendRow(stditem_curr)
            stditem = stditem_curr
        else:
            stditem = stditem_prev

        rospy.logdebug('add_tree_node 1 name_curr=%s ' +
                       '\n\t\t\t\t\tname_prev=%s row_index_parent=%d',
                       name_curr, name_prev, row_index_parent)

        if len(child_names_left) != 0:
            #TODO: View & Model are closely bound here. Ideally isolate this 2.
            #       Maybe we should split into 2 classs, 1 handles view,
            #       the other does model.
            self._add_tree_node(param_item_full, stditem, child_names_left)

    def _refresh_nodes(self):
        #TODO(Isaac) In the future, do NOT remove all nodes. Instead,
        #            remove only the ones that are gone. And add new ones too.

        model = self._rootitem
        if model.hasChildren():
            row_count = model.rowCount()
            model.removeRows(0, row_count)
            rospy.logdebug("ParamWidget _refresh_nodes row_count=%s", row_count)
        self._update_nodetree()

    def close_node(self):
        rospy.logdebug(" in close_node")
        #TODO(Isaac) Figure out if dynamic_reconfigure needs to be closed.
