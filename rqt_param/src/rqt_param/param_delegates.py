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

import rospy
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QModelIndex 
from python_qt_binding.QtGui import QWidget, QStandardItemModel, QItemSelectionModel, QStandardItem, QStyledItemDelegate, QStyle
import dynamic_reconfigure.client
from rqt_py_common.item_delegates import DelegateUtil

from .mvc_editors import StringEditor

## @todo consider moving this to rqt_py_common once it becomes stable.

class ParamDelegate(QStyledItemDelegate):
    """
    
    ROS Parameter as Qt Delegate.
    
    @author: Isaac Saito
    """
    
    def __init__(self, node=None):     
        super(ParamDelegate, self).__init__()
    
    def paint(self, painter, style_option, model_index):        
        """ Paint the items in the table.
            If the item referred to by <index> is a StarRating, we handle the
            painting ourselves. For the other items, we let the base class
            handle the painting as usual.
            In a polished application, we'd use a better check than the 
            column number to find out if we needed to paint the stars, but 
            it works for the purposes of this example.
        """
        
        hierarchy_lv = DelegateUtil.get_hierarchy_level(model_index)
        rospy.logdebug("ParamDelegate paint index.row()=%s, hierarchy=%d", 
                      model_index.row(), hierarchy_lv)
        #if model_index.row() == 2:
        if hierarchy_lv == 3:
            editor = StringEditor(None, None) # TODO args need filled.
            editor.editingFinished.connect(self.commitAndCloseEditor)
            editor.paint(painter, style_option.rect, style_option.palette)
        else:
            QStyledItemDelegate.paint(self, painter, style_option, model_index)

    def paint_org(self, painter, style_option, model_index):        
        """ Paint the items in the table.
            If the item referred to by <index> is a StarRating, we handle the
            painting ourselves. For the other items, we let the base class
            handle the painting as usual.
            In a polished application, we'd use a better check than the 
            column number to find out if we needed to paint the stars, but 
            it works for the purposes of this example.
        """
        hierarchy_lv = DelegateUtil.get_hierarchy_level(model_index)
        rospy.loginfo("paint index.row()=%s, hierarchy=%d", 
                      model_index.row(), hierarchy_lv)
        #if model_index.row() == 2:
        if hierarchy_lv == 3:
            starRating = StarRating(model_index.data())
            # If the row is currently selected, we need to make sure we
            # paint the background accordingly.
            if style_option.state & QStyle.State_Selected:
                # The original C++ example used option.palette.foreground() to
                # get the brush for painting, but there are a couple of 
                # problems with that:
                #   - foreground() is obsolete now, use windowText() instead
                #   - more importantly, windowText() just returns a brush
                #     containing a flat color, where sometimes the style 
                #     would have a nice subtle gradient or something. 
                # Here we just use the brush of the painter object that's
                # passed in to us, which keeps the row highlighting nice
                # and consistent.
                painter.fillRect(style_option.rect, painter.brush())
            # Now that we've painted the background, call starRating.paint()
            # to paint the stars.

            starRating.paint(painter, style_option.rect, style_option.palette)
        else:
            QStyledItemDelegate.paint(self, painter, style_option, model_index)

    def sizeHint(self, style_option, model_index):
        if model_index.column() == 3:
            starRating = StarRating(model_index.data())
            return starRating.sizeHint()
        else:
            return QStyledItemDelegate.sizeHint(self, style_option, model_index)

    def createEditor(self, parent_qw, style_option_view_item, model_index):
        rospy.loginfo('qmindex.row()=%s', model_index.row())
        """ Creates and returns the custom StarEditor object we'll use to edit 
            the StarRating.
        """
        hierarchy_lv = DelegateUtil.get_hierarchy_level(model_index)        
        if hierarchy_lv == 3:
            #editor = StarEditor(parent_qw)
            editor = StringEditor(parent_qw)
            editor.editingFinished.connect(self.commitAndCloseEditor)
            return editor
        else:
            return QStyledItemDelegate.createEditor(self, parent_qw, 
                                                    style_option_view_item, 
                                                    model_index)

    def commitAndCloseEditor(self):
        """ Erm... commits the data and closes the editor. :) """
        editor = self.sender()
        # The commitData signal must be emitted when we've finished editing
        # and need to write our changed back to the model.
        self.commitData.emit(editor)
        self.closeEditor.emit(editor)

    def setEditorData(self, editor, index):
        """ Sets the data to be displayed and edited by our custom editor. """
        if index.row() == 2:
            editor.starRating = StarRating(index.data())
        else:
            QStyledItemDelegate.setEditorData(self, editor, index)
        