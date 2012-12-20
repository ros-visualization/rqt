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
# Author: Isaac Saito, Ze'ev Klapow

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from python_qt_binding.QtGui import QTreeWidgetItem
import rospy

from .inspector_window import InspectorWindow
from .util_robot_monitor import Util
            
class StatusItem(QTreeWidgetItem):
    """
    Represents a single tree node that is capable of holding children objects 
    of its class type.
    """
        
    def __init__(self, status):
        """
        :type status: DiagnosticStatus
        """
        super(StatusItem, self).__init__(QTreeWidgetItem.UserType)
    
        self._children_statusitems = []
        self.name = status.name
        self.level = status.level
        self.last_level = None
        self.inspector = None
        self.status = status # DiagnosticsStatus
        
        self.warning_id = None
        self.error_id = None
        
        self.setText(0, '/' + Util.get_nice_name(self.name))
        
    def get_name(self):
        return self.name

    def enable(self):
        if self.inspector:
            self.inspector.enable()
        
    def disable(self):
        if self.inspector:
            self.inspector.disable()
        
    def update(self, status):
        """
        Replace old status with the passed one.
        
        :type status: DiagnosticsStatus
        """
        self.status = status
        
    def update_children(self, status_new, diag_array):
        """
        
        Recursive for tree node's children.
        Update text on treeWidgetItem, set icon on it.
        
        :type status: DiagnosticStatus
        :type msg: DiagnosticArray
        """
         
        self.status = status_new

        if self.inspector:
            self.inspector.update_status_display(self.status)
        
        children_diag_statuses = Util.get_children(self.name, diag_array)

        names_toplevel_local = [s.name for s in self._children_statusitems]
        new_statusitems = []
        remove = []
        errors = 0
        warnings = 0
        for child_diagnostic_status in children_diag_statuses:
            name = child_diagnostic_status.name
            device_name = Util.get_nice_name(child_diagnostic_status.name)
            headline = "%s : %s" % (child_diagnostic_status.name,
                                    child_diagnostic_status.message)
            
            headline_msg = ''
            if (child_diagnostic_status.level != DiagnosticStatus.OK):
                headline_msg = Util.gen_headline_warn_or_err(child_diagnostic_status)
                if (child_diagnostic_status.level == DiagnosticStatus.ERROR):
                    errors = errors + 1                
                elif (child_diagnostic_status.level == DiagnosticStatus.WARN):
                    warnings = warnings + 1
            else:
               headline_msg = Util.gen_headline_status_green(child_diagnostic_status)
            rospy.logdebug(' update_children level= %s',
                           child_diagnostic_status.level)               
                
            if name in names_toplevel_local:
                index_child = names_toplevel_local.index(name)                
                status_item = self._children_statusitems[ index_child ]
                status_item.update_children(child_diagnostic_status, diag_array)  # Recursive call.
                Util._update_status_images(child_diagnostic_status, status_item)
                rospy.logdebug(' StatusItem update 33 index= %d dev_name= %s',
                               index_child, device_name)
                # status_item.setText(0, headline)
                status_item.setText(0, device_name)
                status_item.setText(1, child_diagnostic_status.message)
            elif len(self.strip_child(name).split('/')) <= 2:
                status_item = StatusItem(child_diagnostic_status)
                status_item.update_children(child_diagnostic_status,
                                            diag_array)  # Recursive call.
                # status_item.setText(0, headline)
                status_item.setText(0, device_name)
                status_item.setText(1, child_diagnostic_status.message)
                self._children_statusitems.append(status_item)
                # new_statusitems.append(status_item)
                self.addChild(status_item)             
 
        rospy.logdebug(' ------ Statusitem.update_children err=%d warn=%d',
                       errors, warnings)
        return {Util._DICTKEY_TIMES_ERROR : errors, 
                Util._DICTKEY_TIMES_WARN : warnings}

    def on_click(self):
        if not self.inspector:
            self.inspector = InspectorWindow(self.status,
                                             self.close_inspector_window)                        
        else:
            self.inspector.activateWindow()
                 
    def close_inspector_window(self):
        rospy.logdebug(' ------ Statusitem close_inspector_window 1')
        self.inspector = None
        
    def strip_child(self, child):
        return child.replace(self.name, '')
    
    def close(self):
        """
        Because Isaac failed to find a way to call a destructor of a class in
        python in general, he made this function, intending it to be called by
        its parent object (in this case RobotMonitorWidget's instance)
        every timeline when a certain node gets removed.        
        """
        if self.inspector:
            # del self.inspector # This doesn't _close the window
            self.inspector.close()
            
        # _close children.
        for status_item in self._children_statusitems:
            status_item.close()
            
class InstantaneousState(object):
    """
    A container for StatusItem per timeframe (second).
    
    Copied from robot_monitor.
    """
    def __init__(self):
        self._items = {} # dict of StatusItem
        self._msg = None
        self._has_warned_no_name = False

    def reset(self):
        self._items = {}
        self._msg = None
        
    def get_parent(self, item):
        parent_name = get_parent_name(item.status.name)
        
        if (parent_name not in self._items):
            return None
        
        return self._items[parent_name]
    
    def get_descendants(self, item):
        rospy.logdebug(' Status get_descendants status.name=%s', 
                       item.status.name)
        child_keys = [k for k in self._items.iterkeys() 
                      if k.startswith(item.status.name + "/")]
        children = [self._items[k] for k in child_keys]
        return children
    
    def get_items(self):
        return self._items
        
    def update(self, msg):
        """        
        Copied from robot_monitor. 
        
        :type msg: DiagnosticArray
        """
        
        removed = []
        added = []
        items = {}
        
        # fill items from new msg, creating new StatusItems for any that don't 
        # already exist, and keeping track of those that have been added new
        for s in msg.status:
            # DiagnosticStatus messages without a name are invalid #3806
            if not s.name and not self._has_warned_no_name:
                rospy.logwarn('DiagnosticStatus message with no "name". ' +
                              'Unable to add to robot monitor. Message: ' +
                              '%s, hardware ID: %s, level: %d' % 
                              (s.message, s.hardware_id, s.level))
                self._has_warned_no_name = True

            if not s.name:
                continue
                
            if (len(s.name) > 0 and s.name[0] != '/'):
                s.name = '/' + s.name

            if (s.name not in self._items):
                i = StatusItem(s)
                added.append(i)
                items[s.name] = i
            else:
                i = self._items[s.name]
                i.update(s)
                items[s.name] = i
        
        # find anything without a parent already in the items, and add it as a 
        # dummy item
        to_add = []
        dummy_names = []
        for i in items.itervalues():
            parent = i.status.name
            while (len(parent) != 0):
                parent = get_parent_name(parent)
                if (len(parent) > 0 and 
                    (parent not in items) and 
                    parent not in dummy_names):
                    
                    pi = None
                    if (parent not in self._items):
                        s = DiagnosticStatus()
                        s.name = parent
                        s.message = ""
                        pi = StatusItem(s)
                    else:
                        pi = self._items[parent]
                        
                    to_add.append(pi)
                    dummy_names.append(pi.status.name)
                  
        for a in to_add:
            if (a.status.name not in items):
                items[a.status.name] = a
                
                if (a.status.name not in self._items):
                    added.append(a)
        
        for i in self._items.itervalues():
            # determine removed items
            if (i.status.name not in items):
                removed.append(i)
        
        # remove removed items
        for r in removed:
            del self._items[r.status.name]
        
        self._items = items
        self._msg = msg
        
        # sort so that parents are always added before children
        added.sort(cmp=lambda l,r: cmp(l.status.name, r.status.name))
        # sort so that children are always removed before parents
        removed.sort(cmp=lambda l,
                     r: cmp(l.status.name, r.status.name), 
                     reverse=True)
        
        return (added, removed, self._items)
            