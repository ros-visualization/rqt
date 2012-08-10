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

PKG = 'rxbag_plugins'
import roslib; roslib.load_manifest(PKG)
import rospy

import codecs
import collections
import string
import threading
import time

import numpy
import wx

class PlotConfigureFrame(wx.Frame):
    """
    Choose data to extract from messages.
    """
    def __init__(self, plot):
        wx.Frame.__init__(self, None, title=plot.parent.Title + ' - Configure', size=(600, 400))

        self.plot = plot

        splitter = wx.SplitterWindow(self)
        
        self._drag_item = None

        self.font = wx.Font(9, wx.FONTFAMILY_MODERN, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL)

        self.msg_tree = wx.TreeCtrl(splitter, style=wx.TR_DEFAULT_STYLE | wx.TR_HIDE_ROOT)
        self.add_msg_object(None, '', 'msg', self.plot._message, self.plot._message._type)
        self.msg_tree.Bind(wx.EVT_LEFT_DCLICK, self.on_msg_left_dclick)

        self.plot_tree = wx.TreeCtrl(splitter, style=wx.TR_DEFAULT_STYLE | wx.TR_HIDE_ROOT | wx.TR_HAS_BUTTONS)
        
        # Disabling dragging of messages between plots - caused weird behavior whereby top frame won't receive EVT_CLOSE after drag event is Allow'ed
        #self.plot_tree.Bind(wx.EVT_TREE_BEGIN_DRAG, self.on_plot_begin_drag)
        #self.plot_tree.Bind(wx.EVT_TREE_END_DRAG,   self.on_plot_end_drag)
        
        self.plot_tree.Bind(wx.EVT_LEFT_DCLICK,     self.on_plot_left_dclick)

        size = (16, 16)
        self.imagelist = wx.ImageList(*size)
        self.plot_image_index = self.imagelist.Add(wx.Bitmap(roslib.packages.get_pkg_dir(PKG) + '/icons/chart_line.png'))
        self.plot_tree.SetImageList(self.imagelist)

        splitter.SplitVertically(self.msg_tree, self.plot_tree)
        splitter.SashPosition    = 350
        splitter.MinimumPaneSize = 100

        self.Bind(wx.EVT_CLOSE, self.on_close)

        self._create_toolbar()

        #

        self.plot_root = self.plot_tree.AddRoot('Plots')

        self.plot_items = []
        if len(self.plot.plot_paths) > 0:
            for plot in self.plot.plot_paths:
                self.plot_items.append(self.add_plot())
                for path in plot:
                    self.plot_add_msg(path)
        else:
            self.add_plot()
            self.plot_tree.SelectItem(self.plot_items[0])

        self._update_msg_tree()

    def on_close(self, event):
        self.plot._configure_frame = None
        event.Skip()

    def _create_toolbar(self):
        icons_dir = roslib.packages.get_pkg_dir(PKG) + '/icons/'

        tb = self.CreateToolBar()
        tb.Bind(wx.EVT_TOOL, lambda e: self.plot_add_selected_msg(),    tb.AddLabelTool(wx.ID_ANY, '', wx.Bitmap(icons_dir + 'chart_line_add.png')))
        tb.Bind(wx.EVT_TOOL, lambda e: self.plot_delete_selected_msg(), tb.AddLabelTool(wx.ID_ANY, '', wx.Bitmap(icons_dir + 'chart_line_delete.png')))
        tb.Bind(wx.EVT_TOOL, lambda e: self.add_plot(),                 tb.AddLabelTool(wx.ID_ANY, '', wx.Bitmap(icons_dir + 'add.png')))
        tb.Bind(wx.EVT_TOOL, lambda e: self.delete_plot(),              tb.AddLabelTool(wx.ID_ANY, '', wx.Bitmap(icons_dir + 'delete.png')))
        tb.Realize()

    def add_msg_object(self, parent, path, name, obj, obj_type):
        label = name
        
        if hasattr(obj, '__slots__'):
            subobjs = [(slot, getattr(obj, slot)) for slot in obj.__slots__]
        elif type(obj) is list or type(obj) is tuple:
            subobjs = [('[%d]' % i, subobj) for (i, subobj) in enumerate(obj)]
        else:
            subobjs = []
            
            # Ignore any binary data
            obj_repr = codecs.utf_8_decode(str(obj), 'ignore')[0]
            
            # Truncate long representations
            if len(obj_repr) >= 50:
                obj_repr = obj_repr[:50] + '...'
            
            label += ': ' + obj_repr

        if parent is None:
            item = self.msg_tree.AddRoot(label)
        else:
            item = self.msg_tree.AppendItem(parent, label)

        self.msg_tree.SetItemFont(item, self.font)
        self.msg_tree.SetItemPyData(item, (path, obj_type))

        if self.msg_item_is_plottable(item):
            self.msg_tree.SetItemTextColour(item, wx.Colour(0, 0, 0))
        else:
            self.msg_tree.SetItemTextColour(item, wx.Colour(100, 100, 100))

        for subobj_name, subobj in subobjs:
            if subobj is None:
                continue
            
            if path == '':
                subpath = subobj_name                       # root field
            elif subobj_name.startswith('['):
                subpath = '%s%s' % (path, subobj_name)      # list, dict, or tuple
            else:
                subpath = '%s.%s' % (path, subobj_name)     # attribute (prefix with '.')

            if hasattr(subobj, '_type'):
                subobj_type = subobj._type
            else:
                subobj_type = type(subobj).__name__

            self.add_msg_object(item, subpath, subobj_name, subobj, subobj_type)

    def msg_item_is_plottable(self, item):
        (path, obj_type) = self.msg_tree.GetItemPyData(item)

        return obj_type in ['int', 'float', 'long']

    ### Plots ###

    def on_plot_left_dclick(self, event):
        self.plot_delete_selected_msg()

    def plot_delete_selected_msg(self):
        selected_item = self.plot_tree.Selection
        if not selected_item.IsOk() or selected_item in self.plot_items:
            return

        self.plot_tree.Delete(selected_item)

        self._update_msg_tree()

    def get_plot_paths(self):
        items = []
        PlotConfigureFrame.traverse(self.plot_tree, self.plot_tree.RootItem, items.append)
        
        plot_paths = []
        for item in items:
            if item == self.plot_tree.RootItem:
                continue
            
            text = str(self.plot_tree.GetItemText(item))
            
            if item in self.plot_items:
                plot_paths.append([])
            elif len(plot_paths) > 0:
                plot_paths[-1].append(text)

        return plot_paths

    def add_plot(self):
        item = self.plot_tree.AppendItem(self.plot_root, 'Plot')
        self.plot_tree.SetItemImage(item, self.plot_image_index, wx.TreeItemIcon_Normal)
        self.plot_items.append(item)
        
        self._update_msg_tree()
        
        self.plot_tree.SelectItem(item)

        return item

    def delete_plot(self):
        selected_item = self.plot_tree.Selection
        if not selected_item.IsOk() or selected_item not in self.plot_items:
            return

        self.plot_tree.Delete(selected_item)
        self.plot_items.remove(selected_item)

        self._update_msg_tree()

    ### Message ###
    
    def on_msg_left_dclick(self, event):
        self.plot_add_selected_msg()

    def plot_add_selected_msg(self):
        if len(self.plot_items) == 0:
            return
        selected_item = self.msg_tree.Selection
        if not selected_item.IsOk() or not self.msg_item_is_plottable(selected_item):
            return

        (path, obj_type) = self.msg_tree.GetItemPyData(selected_item)

        self.plot_add_msg(path)

    def plot_add_msg(self, path):
        selected_plot = self.plot_tree.Selection
        if not selected_plot.IsOk() or selected_plot == self.plot_tree.RootItem:
            selected_plot = self.plot_items[-1]

        self.plot_tree.AppendItem(selected_plot, path)
        self.plot_tree.ExpandAll()

        self._update_msg_tree()

    def _update_msg_tree(self):
        plot_paths = self.get_plot_paths()
        
        if plot_paths != self.plot.plot_paths:
            self.plot.plot_paths = plot_paths

        PlotConfigureFrame.traverse(self.msg_tree, self.msg_tree.RootItem, self._update_msg_tree_item)

    def _update_msg_tree_item(self, item):
        (path, obj_type) = self.msg_tree.GetItemPyData(item)

        plotted = False
        for paths in self.plot.plot_paths:
            if path in paths:
                plotted = True
                break

        self.msg_tree.SetItemBold(item, plotted)

    def on_plot_begin_drag(self, event):
        if not self.plot_tree.ItemHasChildren(event.Item):
            self._drag_item = event.Item
            event.Allow()

    def on_plot_end_drag(self, event):
        dest = event.Item

        src = self._drag_item

        if src is None or dest == src:
            return

        text = self.plot_tree.GetItemText(src)
        data = self.plot_tree.GetItemPyData(src)

        self.plot_tree.Delete(src)

        if dest.IsOk():
            is_plot_item = (self.plot_tree.GetItemParent(dest) == self.plot_tree.RootItem) 
            if is_plot_item:
                new_item = self.plot_tree.InsertItemBefore(dest, 0, text)
            else:
                parent = self.plot_tree.GetItemParent(dest)
                if not parent.IsOk():
                    return
                new_item = self.plot_tree.InsertItem(parent, dest, text)

            self.plot_tree.SetItemPyData(new_item, data)

        self.plot_tree.ExpandAll()

        self._update_msg_tree()

    @staticmethod
    def traverse(tree, root, function):
        if tree.ItemHasChildren(root):
            first_child = tree.GetFirstChild(root)[0]
            function(first_child)
            PlotConfigureFrame.traverse(tree, first_child, function)

        child = tree.GetNextSibling(root)
        if child:
            function(child)
            PlotConfigureFrame.traverse(tree, child, function)
