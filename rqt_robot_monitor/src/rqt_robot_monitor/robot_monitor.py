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

import os
import copy
import random
import pdb

import roslib;roslib.load_manifest('rqt_robot_monitor')
import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget, QTreeWidgetItem, QTextEdit, QIcon
from python_qt_binding.QtCore import Signal, Qt

from inspector_window import InspectorWindow
from timeline_pane import TimelinePane

_ERR_LEVELS = [2, 3]

# Instantiating icons that show the device status.
_ERR_ICON = QIcon.fromTheme('face-angry')
_WARN_ICON = QIcon.fromTheme('face-sick')
_OK_ICON = QIcon.fromTheme('face-laugh')  
_STALE_ICON = QIcon.fromTheme('face-tired')  # Added following this QA thread
                                            # http://goo.gl/83tVZ
_IMG_DICT = {0: _OK_ICON, 1: _WARN_ICON, 2: _ERR_ICON, 3: _STALE_ICON}

'''
TODO Following non-class functions need to be considered about porting out to 
     common class that is accessible from other packages. 
'''
'''
@param param: status_name is a string that may consists of status names that 
              are delimited by slash.   
@return: string
'''
def get_nice_name(status_name):
    return status_name.split('/')[-1]

def remove_parent_name(status_name):
    return ('/'.join(status_name.split('/')[2:])).strip()

def get_parent_name(status_name):
    return ('/'.join(status_name.split('/')[:-1])).strip()

def gen_headline_status_green(diagnostic_status):
    # return "%s : %s" % (get_nice_name(diagnostic_status.status.name), 
    #                                   diagnostic_status.status.message)
    return "%s" % get_nice_name(diagnostic_status.name)

def gen_headline_warn_or_err(diagnostic_status):
    return "%s : %s" % (get_nice_name(diagnostic_status.name),
                        diagnostic_status.message)
    
'''
Taken from robot_monitor.robot_monitor_panel.py
@param status: DiagnosticStatus 
@param node: StatusItem 
@author: Isaac Saito 
'''
def update_status_images(diagnostic_status, statusitem):
    name = diagnostic_status.name
    if (name is not None):
        # level = diagnosis_status.level
        level = diagnostic_status.level                
        rospy.logdebug('New diagnostic_status level: %s. Last lv: %s name: %s',
                       level, statusitem.last_level, name)        
        if (diagnostic_status.level != statusitem.last_level):  
            # TODO Apparently diagnosis_status doesn't contain last_level. 
            statusitem.setIcon(0, _IMG_DICT[level])
            statusitem.last_level = level
            return
              
'''
Represents a single node on tree that can have multiple children objects of 
its class type.
'''              
class StatusItem(QTreeWidgetItem):
    '''
    @param status: DiagnosticStatus 
    '''
    def __init__(self, status):
        super(StatusItem, self).__init__(QTreeWidgetItem.UserType)
    
        self._children_statusitems = []
        self.name = status.name
        self.level = status.level
        self.last_level = None
        self.inspector = None
        self.status = status
        
        self.warning_id = None
        self.error_id = None
        
        self.setText(0, '/' + get_nice_name(self.name))

    '''
    @param msg: DiagnosticArray 
    @return: DiagnosticStatus[]
    '''
    def _get_children(self, diag_array):
        ret = []
        for k in diag_array.status:  # k is DiagnosticStatus. 
            if k.name.startswith(self.name):  # Starting with self.name means k 
                                       # is either top/parent node or its child.
                if not k.name == self.name:  # Child's name must be different 
                                            # from that of the top/parent node.  
                    ret.append(k)
        return ret

    '''
    Recursive for tree node's children.
    Update text on treeWidgetItem, set icon on it.
    
    @param status: DiagnosticStatus 
    @param msg: DiagnosticArray 
    '''
    def update_children(self, diag_status, diag_array):
        self.status = diag_status

        if self.inspector:
            self.inspector.update_children(diag_status)
        
        children_diag_statuses = self._get_children(diag_array)

        names_toplevel_local = [s.name for s in self._children_statusitems]
        new_statusitems = []
        remove = []
        errors = 0
        warnings = 0
        for child_diagnostic_status in children_diag_statuses:
            name = child_diagnostic_status.name
            device_name = get_nice_name(child_diagnostic_status.name)
            headline = "%s : %s" % (child_diagnostic_status.name,
                                    child_diagnostic_status.message)
            
            headline_msg = ''
            if (child_diagnostic_status.level != DiagnosticStatus.OK):
                headline_msg = gen_headline_warn_or_err(child_diagnostic_status)
                if (child_diagnostic_status.level == DiagnosticStatus.ERROR):
                    errors = errors + 1                
                elif (child_diagnostic_status.level == DiagnosticStatus.WARN):
                    warnings = warnings + 1
            else:
               headline_msg = gen_headline_status_green(child_diagnostic_status)
            rospy.logdebug(' update_children level= %s',
                           child_diagnostic_status.level)               
                
            if name in names_toplevel_local:
                index_child = names_toplevel_local.index(name)                
                status_item = self._children_statusitems[ index_child ]
                status_item.update_children(child_diagnostic_status,
                                            diag_array)  # Recursive call.
                update_status_images(child_diagnostic_status, status_item)
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
                      
        # self.addChildren(new_statusitems) #QTreeWidgetItem::addChildren(QList)
        # return set/dict of error&warn
        
        rospy.logdebug(' ------ Statusitem.update_children err=%d warn=%d',
                       errors, warnings)
        return { 'times_errors' : errors, 'times_warnings' : warnings }

    def on_click(self):
        if not self.inspector:
            rospy.logdebug('StatusItem.on_click No InspectorWindow Found')
            self.inspector = InspectorWindow(self.status)                        
            self.inspector.sig_close_window.connect(self.close_inspector_window)            
        else:
            rospy.logdebug('StatusItem.on_click activate InspectorWindow')
            self.inspector.activateWindow()
            
    '''
    @author: Isaac Saito
    '''        
    def close_inspector_window(self):
        self.inspector = None
        
    def strip_child(self, child):
        return child.replace(self.name, '')
    
    '''
    Because Isaac failed to find a way to call a destructor of a class in 
     python in general, he made this function, intending it to be called by 
     its parent object (in this case RobotMonitorWidget's instance) 
     every timeline when a certain node gets removed.
    
    @author: Isaac Saito
    '''        
    def close(self):
        if self.inspector:
            # del self.inspector # Doesn't _close the window
            self.inspector.close()
            
        # _close children.
        for status_item in self._children_statusitems:
            status_item.close()
            
'''
shutdown function needs to be called when the class terminates.
'''            
class RobotMonitorWidget(QWidget):
    # sig_err = Signal(str, int)
    # sig_warn = Signal(str, str, int)    
    _sig_clear = Signal()

    def __init__(self, context, topic):
        super(RobotMonitorWidget, self).__init__()
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)),
                               'rqt_robot_monitor_mainwidget.ui')
        loadUi(ui_file, self)
        
        obj_name = 'Robot Monitor'
        self.setObjectName(obj_name)
        
        self._toplv_statusitems = []  # StatusItem
        self._warn_statusitems = []  # StatusItem. Contains ALL DEGREES 
                                 # (device top level, device' _sub) in parallel. 
        self._err_statusitems = []  # StatusItem
        
        self._sig_clear.connect(self._clear)
        
        self.tree_all_devices.itemDoubleClicked.connect(self._tree_clicked)
        self.warn_tree.itemDoubleClicked.connect(self._tree_clicked)
        self.err_tree.itemDoubleClicked.connect(self._tree_clicked)
        
        self.tree_all_devices.resizeColumnToContents(0)
        # self.tree_all_devices.resizeColumnToContents(1)
        # self.tree_all_devices.sortByColumn(0, Qt.AscendingOrder) 
        #      No effect - sorting didn't get enabled.
        
        # TODO Declaring timeline pane. 
        #      Needs to be stashed away into .ui file but so far failed. 
        self.timeline_pane = TimelinePane(self)
        # self.timeline_pane.setMinimumSize(100, 50)
        # self.splitter.insertWidget(3, self.timeline_pane)
        self.vlayout_top.addWidget(self.timeline_pane)                

        self.paused = False

        self.setWindowTitle('Robot Monitor')   
             
        self.num_diag_msg_received = 0  # For debug
        
        self.topic = topic
        self._sub = rospy.Subscriber(
                                    self.topic,  # name of the topic 
                                    DiagnosticArray,  # type of the topic
                                    self._cb)        
        
    def _cb(self, msg):        
        if not self.paused:            
            # self._sig_clear.emit()
            rospy.logdebug('_cb 00')
            self._update_devices_tree(msg)
            self._update_warns_errors(msg)
            rospy.logdebug('_cb 22')
            self.timeline_pane.add_message(msg)            
        
        self.num_diag_msg_received += 1
        rospy.logdebug('  RobotMonitorWidget _cb #%d',
                       self.num_diag_msg_received)
        
    '''
    @param item: QTreeWidgetItem 
    @param column: int 
    '''
    def _tree_clicked(self, item, column):
        rospy.logdebug('RobotMonitorWidget _tree_clicked col=%d', column) 
        item.on_click()
        
    '''
    Update the tree from the bottom    
    @param diag_array: DiagnosticArray 

    @todo: 11/5/2012 Currently, in case some devices disappear 
                     while running this program, there's no way to remove 
                     those from the device-tree. 
    '''
    def _update_devices_tree(self, diag_array):
        statusnames_curr_toplevel = [get_nice_name(k.name) 
                                     for k in self._toplv_statusitems]  
        # Only the k variable that pops up at the end is 
        # processed by get_nice_name.
        
        for diagnostic_status_new in self._get_toplv_diagnosticstatus_from_new_msg(diag_array):
            name = get_nice_name(diagnostic_status_new.name)
            rospy.logdebug('_update_devices_tree 0 name @ toplevel %s', name)
            dict_status = 0
            if name in statusnames_curr_toplevel:  # No change of names 
                                                 # in toplevel since last time. 
                statusitem = self._toplv_statusitems[
                                        statusnames_curr_toplevel.index(name)]
                
                # TODO 10/30/2012/Isaac/Next lines (where status update_
                #                       children occurs) can cause flickering 
                #                       as the number of nodes increase.
                #                       Can refer to orig robot_model for ideas
                dict_status = statusitem.update_children(diagnostic_status_new,
                                                         diag_array)
                times_errors = dict_status['times_errors']
                times_warnings = dict_status['times_warnings']
                update_status_images(diagnostic_status_new, statusitem)
                                                
                # TODO Update status text on each node using dict_status.
                base_text = gen_headline_status_green(statusitem.status)
                # errwarn_text = gen_headline_warn_or_err(statusitem.status)
 
                rospy.logdebug('_update_devices_tree warn_id= %s\n\t\t\t' + 
                               'diagnostic_status.name = %s\n\t\t\t\t' + 
                               'Normal status diag_array = %s',
                               statusitem.warning_id,
                               diagnostic_status_new.name, base_text)
                
                if (times_errors > 0 or times_warnings > 0):                    
                    base_text = "(Err: %s, Wrn: %s) %s %s" % (
                                   times_errors,
                                   times_warnings,
                                   get_nice_name(diagnostic_status_new.name),
                                   diagnostic_status_new.message)
                    rospy.logdebug('_update_dev_tree 111 text to show=%s',
                                   base_text)
                    statusitem.setText(0, base_text)
                    statusitem.setText(1, diagnostic_status_new.message)
                else:
                    rospy.logdebug('_update_dev_tree 222 text to show=%s',
                                   base_text)
                    statusitem.setText(0, base_text)
                    statusitem.setText(1, '-')
       
            else:
                new_status_item = StatusItem(diagnostic_status_new)
     
                # TODO receive return value set and use them.
                new_status_item.update_children(diagnostic_status_new,
                                                diag_array)
                
                # TODO Figure out if a statusitem and 
                #      its subtree contains errors.
                # new_status_item.setIcon(0, self._error_icon) 
                #      This shows NG icon at the beginning of each statusitem.
                update_status_images(diagnostic_status_new, new_status_item)
                
                self._toplv_statusitems.append(new_status_item)
                
                # TODO new_statusitems_toplv might not be necessary - 
                #                        _toplv_statusitems might substitute.
                # new_statusitems_toplv.append(new_status_item)
                rospy.logdebug(' _update_devices_tree 2 ' + 
                               'diagnostic_status_new.name %s',
                               new_status_item.name)
                self.tree_all_devices.addTopLevelItem(new_status_item)
        # self.tree_all_devices.addTopLevelItems(new_statusitems_toplv)
        
    '''
    Return an array that contains DiagnosticStatus only at the top level of 
    the given msg.
      
    @param msg: DiagnosticArray 
    @return array of 'status' (ie. array of DiagnosticStatus[])
    '''
    def _get_toplv_diagnosticstatus_from_new_msg(self, diag_array):
        ret = []
        for diagnostic_status in diag_array.status:
            if len(diagnostic_status.name.split('/')) == 2:
                rospy.logdebug(" _get_toplv_diagnosticstatus_from_new_msg " + 
                "TOP lev %s ", diagnostic_status.name)
                ret.append(diagnostic_status)        
            else:
                rospy.logdebug(" _get_toplv_diagnosticstatus_from_new_msg " + 
                               "Not top lev %s ", diagnostic_status.name)
        return ret

    '''
    @param key: string
    @param statusitems: DiagnosticStatus[]  
    @return: int of index that key is found in array. -1 if not found
    '''
    def _contains(self, key, statusitems):
        names = [get_nice_name(k.name) for k in statusitems]
        
        rospy.logdebug('\t_contains len of input = %d', len(statusitems))
        for elem in names:  # This loop is only for debug 
            rospy.logdebug('\t_contains CONTAINED KEY= %s', elem)
        
        if key in names:
            rospy.logdebug('** _contains key IS contained.')
            return names.index(key)
        else:
            rospy.logdebug('** _contains key IS NOT contained.')
            return -1 
        
    """
    Update the warning and error trees. 
    
    Unlike _update_devices_tree function where all DiagnosticStatus need to be 
    inspected constantly, this function is used in a trial to reduce 
    unnecessary inspection of the status level for all DiagnosticStatus 
    contained in the incoming DiagnosticArray msg.
    
    @param msg: DiagnosticArray 
    """
    def _update_warns_errors(self, diag_array):
        # self._update_flat_tree_3(diag_array, self._toplv_statusitems)
        self._update_flat_tree(diag_array)
       
    def pause(self, msg):
        self.paused = True
        # self._sig_clear.emit()
        # self._update_devices_tree(msg)
        # self._update_warns_errors(msg)
        
    def unpause(self):
        self.paused = False

    '''
    11/5/2012/Isaac/Not really called from anywhere.
    '''
    def _clear(self):
        rospy.logdebug(' RobotMonitorWidget _clear called ')
        self.err_tree.clear()
        self.warn_tree.clear()
    
    '''
    Added 11/12/2012 Still buggy
    '''
    def _update_flat_tree_3(self, diag_arr, statusitems_curr_toplevel):
        devicenames_toplevel_curr = [get_nice_name(k.name) 
                                     for k in statusitems_curr_toplevel]
        for diag_stat_new in diag_arr.status:
            stat_lv_new = diag_stat_new.level
            dev_name = diag_stat_new.name
            dev_index_warn_curr = self._contains(dev_name,
                                                 self._warn_statusitems)
            dev_index_err_curr = self._contains(dev_name, self._err_statusitems)
            headline = "%s" % diag_stat_new.name
            if DiagnosticStatus.OK == stat_lv_new:
                if 0 <= dev_index_warn_curr:
                    statitem_curr = self._remove_statitem(dev_index_warn_curr,
                                                          self._warn_statusitems,
                                                          self.warn_tree)
                    statitem_curr.warning_id = None                    
                elif 0 <= dev_index_err_curr:
                    statitem_curr = self._remove_statitem(dev_index_err_curr,
                                                          self._err_statusitems,
                                                          self.err_tree)
                    statitem_curr.error_id = None
            elif DiagnosticStatus.WARN == stat_lv_new:
                if 0 <= dev_index_err_curr:
                    statitem_curr = self._remove_statitem(dev_index_err_curr,
                                                          self._err_statusitems,
                                                          self.err_tree)
                    self._add_statitem(statitem_curr, self._warn_statusitems,
                                       self.warn_tree,
                                       headline, diag_stat_new.message,
                                       stat_lv_new)
                elif (dev_index_warn_curr < 0 and dev_index_err_curr < 0):
                    statitem_new = StatusItem(diag_stat_new)
                    self._add_statitem(statitem_new, self._warn_statusitems,
                                       self.warn_tree, headline,
                                       diag_stat_new.message, stat_lv_new)
            elif DiagnosticStatus.ERROR == stat_lv_new:
                if 0 <= dev_index_warn_curr:
                    statitem_curr = self._remove_statitem(dev_index_warn_curr,
                                                          self._warn_statusitems,
                                                          self.warn_tree)
                    self._add_statitem(statitem_curr, self._err_statusitems,
                                       self.err_tree, headline,
                                       diag_stat_new.message, stat_lv_new)
                elif (dev_index_warn_curr < 0 and dev_index_err_curr < 0):
                    statitem_new = StatusItem(diag_stat_new)
                    self._add_statitem(statitem_new, self._err_statusitems,
                                       self.err_tree, headline,
                                       diag_stat_new.message, stat_lv_new)

    '''
    @author: Isaac Saito
    '''
    def _add_statitem(self, statusitem, statitem_list, tree, headline,
                      statusmsg, statlevel):
        statusitem.setText(0, headline)
        statusitem.setText(1, statusmsg)
        statusitem.setIcon(0, _IMG_DICT[statlevel])
        statitem_list.append(statusitem)                
        tree.addTopLevelItem(statusitem)
        
    '''
    @author: Isaac Saito
    '''            
    def _remove_statitem(self, item_index, item_list, tree):
        statitem_existing = item_list[item_index]
        tree.takeTopLevelItem(tree.indexOfTopLevelItem(statitem_existing))
        item_list.pop(item_index)
        return statitem_existing

        
    '''
    Update the given flat tree (that doesn't show children. 
    All of its elements are top level) 
    with all the DiagnosticStatus instances contained in the given 
    DiagnosticArray, regardless of the degree of the device.
    
    @param diag_arr: DiagnosticArray class.
    @author: Isaac Saito
    '''
    def _update_flat_tree(self, diag_arr):
        statusnames_curr_toplevel = [get_nice_name(k.name) for k 
                                     in self._toplv_statusitems]
        for diag_stat_new in diag_arr.status:
            # Children of toplevel items are taken care of, 
            #  by examining all DiagnosticStatus array elements 
            #  in DiagnosticArray.
            level = diag_stat_new.level
            dev_name = get_nice_name(diag_stat_new.name)            
            if dev_name in statusnames_curr_toplevel:
                continue  # Skipping top level device to be shown (all device 
                          # tree always shows it, so no need to show them on 
                          # warn / err trees). 
            
            statitems_existing = []
            itemtree = None            
            if DiagnosticStatus.WARN == level:
                 itemtree = self.warn_tree
            elif DiagnosticStatus.ERROR == level:
                 itemtree = self.err_tree
                
            dev_index = self._contains(dev_name, self._warn_statusitems)
            rospy.logdebug(' 1 _update_flat_tree dev_index=%d', dev_index)
            if 0 <= dev_index:
                statitems_existing = self._warn_statusitems
            elif dev_index < 0: 
                dev_index = self._contains(dev_name, self._err_statusitems)
                rospy.logdebug(' 2 _update_flat_tree dev_index=%d', dev_index)
                if 0 <= dev_index:
                    statitems_existing = self._err_statusitems
                else:
                    if DiagnosticStatus.WARN == level:
                        statitems_existing = self._warn_statusitems
                    elif DiagnosticStatus.ERROR == level:
                        statitems_existing = self._err_statusitems
            rospy.logdebug('    _update_flat_tree statusitem.lev=%s ' + 
                           'dev_name=%s dev_index=%d',
                           level, dev_name, dev_index)
            
            # headline = "%s : %s" % (diag_stat_new.name, diag_stat_new.message)
            headline = "%s" % diag_stat_new.name
            
            if 0 <= dev_index:  # Not a new device for warn tree. 
                statitem_existing = statitems_existing[dev_index]
                
#                item_id = {
#                    DiagnosticStatus.WARN: lambda: statitem_existing.warning_id,
#                    DiagnosticStatus.ERROR: lambda: statitem_existing.error_id,
#                }[diagstat_lev]()
                rospy.logdebug(' _update_flat_tree statusitem.lev=%s ' + 
                               'warn_id=%s err_id=%s name=%s', level,
                               statitem_existing.warning_id,
                               statitem_existing.error_id,
                               statitem_existing.name)
                
                if (DiagnosticStatus.WARN != level and 
                    statitem_existing.warning_id is not None):
                    rospy.logdebug(' _update_flat_tree REMOVE name=%s',
                                   statitem_existing.name)                    
                    # self.warn_tree.removeItemWidget(statitem_existing, 0) 
                    #   removeItemWidget doesn't remove an item from tree. 
                    self.warn_tree.takeTopLevelItem(
                          self.warn_tree.indexOfTopLevelItem(statitem_existing))
                    # statitems_existing.remove(statitem_existing) 
                    #    pyside causes error with this (NotImplementedError: 
                    #                     operator not implemented.)
                    self._warn_statusitems.pop(dev_index)
                    statitem_existing.warning_id = None
                
                # Remove ERROR    
                elif (DiagnosticStatus.ERROR != level and 
                      statitem_existing.error_id is not None):
                    rospy.logdebug(' err REMOVE FROM TREE name=%s',
                                   statitem_existing.name)
                    # self.warn_tree.removeItemWidget(statitem_existing, 0) 
                    #     removeItemWidget doesn't remove an item from tree. 
                    self.err_tree.takeTopLevelItem(
                          self.err_tree.indexOfTopLevelItem(statitem_existing))  
                    # statitems_existing.remove(statitem_existing) 
                    #    pyside causes error here 
                    #    (NotImplementedError: operator not implemented.)
                    self._err_statusitems.pop(dev_index)
                    # del statitem_existing 
                    #    Trying to delete in nested clause will
                    #    cause terrible error.
                    statitem_existing.error_id = None
                     
                else:
                    rospy.logdebug('** _update_flat_tree statusitem. ELSE ')
                           
            elif dev_index < 0 and 0 < level:  # This statusitem does not exist, 
                                               # and status is not "OK". 
                # New device for warn tree. Create new statusitem instance.
                statitem_new = StatusItem(diag_stat_new)
                if DiagnosticStatus.WARN == level:
                    statitem_new.warning_id = random.random()
                elif DiagnosticStatus.ERROR == level:
                    statitem_new.error_id = random.random()
                # statitem_new.setText(0, headline)
                rospy.logdebug(' NEW _update_warning_tree new.name= %s,' + 
                               'diag_stat_new.msg= %s',
                              diag_stat_new.name, diag_stat_new.message)
                statitem_new.setText(0, headline)
                statitem_new.setText(1, diag_stat_new.message)
                statitem_new.setIcon(0, _IMG_DICT[level])
                # all_lev_statitems_tobe_shown.append(statitem_new)                
                statitems_existing.append(statitem_new)                
                itemtree.addTopLevelItem(statitem_new)
            
    def _close(self):  # 10/24/Isaac/When this is called?
        rospy.logdebug('RobotMonitorWidget in _close')
        if self._sub:
            self._sub.unregister()
            self._sub = None

    '''
    This needs to be called whenever this class terminates.
    '''
    def shutdown(self):
        rospy.logdebug('RobotMonitorWidget in _shutdown')
        # Close all StatusItem (and each associated InspectWidget)        
        # self.tree_all_devices.clear()  # Doesn't work for the purpose 
                                         # (inspector windows don't get closed)
              
        for item in self._err_statusitems:
            item.close()
        for item in self._warn_statusitems:
            item.close()
        for item in self._toplv_statusitems:
            item.close()

        # unsubscribe from Topics
        self._sub.unregister()

        # stop timers --> no timer in use.
        
