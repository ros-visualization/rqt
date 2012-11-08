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

error_levels = [2, 3]

# Instantiating icons that show the device status.
error_icon = QIcon.fromTheme('face-angry')
warning_icon = QIcon.fromTheme('face-sick')
ok_icon = QIcon.fromTheme('face-laugh')  
stale_icon = QIcon.fromTheme('face-tired')  # Added following this QA thread http://goo.gl/83tVZ
image_dict = { 0: ok_icon, 1: warning_icon, 2: error_icon, 3: stale_icon }

'''
@param param: status_name is a string that may consists of status names that are delimited by slash.   
@return: string
'''
def get_nice_name(status_name):
    return status_name.split('/')[-1]

def remove_parent_name(status_name):
    return ('/'.join(status_name.split('/')[2:])).strip()

def get_parent_name(status_name):
    return ('/'.join(status_name.split('/')[:-1])).strip()

def gen_headline_status_green(diagnostic_status):
    # return "%s : %s" % (get_nice_name(diagnostic_status.status.name), diagnostic_status.status.message)
    return "%s : %s" % (get_nice_name(diagnostic_status.name), diagnostic_status.message)

def gen_headline_warn_or_err(diagnostic_status):
    return "%s : %s" % (get_nice_name(diagnostic_status.name), diagnostic_status.message)
    
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
        rospy.logdebug('New diagnostic_status level: %s. Last level: %s name: %s', level, statusitem.last_level, name)
        # if (diagnosis_status.level != statusitem.last_level):  # TODO Apparently diagnosis_status doesn't contain last_level.
        if (diagnostic_status.level != statusitem.last_level):  # TODO Apparently diagnosis_status doesn't contain last_level. 
        # self._tree_ctrl.SetItemImage(item.tree_id, self._image_dict[level])
            statusitem.setIcon(0, image_dict[level])
            statusitem.last_level = level
            return
              
'''
Represents a single node on tree that can have multiple children objects of its class type.
'''              
class StatusItem(QTreeWidgetItem):
    '''
    @param status: DiagnosticStatus 
    '''
    def __init__(self, status):
        super(StatusItem, self).__init__()
    
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
        for k in diag_array.status:  # k is DiagnosticStatus. See http://www.ros.org/doc/api/diagnostic_msgs/html/diag_array/DiagnosticArray.html
            if k.name.startswith(self.name):  # Starting with self.name means k is either top/parent node or its child.
                if not k.name == self.name:  # Child's name must be different from that of the top/parent node.  
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
            headline = "%s : %s" % (child_diagnostic_status.name, child_diagnostic_status.message)
            
            headline_msg = ''
            if (child_diagnostic_status.level != DiagnosticStatus.OK):
                headline_msg = gen_headline_warn_or_err(child_diagnostic_status)
                if (child_diagnostic_status.level == DiagnosticStatus.ERROR):
                    errors = errors + 1                
                elif (child_diagnostic_status.level == DiagnosticStatus.WARN):
                    warnings = warnings + 1
            else:
               headline_msg = gen_headline_status_green(child_diagnostic_status)
            rospy.logdebug(' update_children level= %s', child_diagnostic_status.level)               
                
            if name in names_toplevel_local:
                index_child = names_toplevel_local.index(name)                
                status_item = self._children_statusitems[ index_child ]
                status_item.update_children(child_diagnostic_status, diag_array)  # Recursive call.
                update_status_images(child_diagnostic_status, status_item)
                rospy.logdebug(' StatusItem update 33 index= %d dev_name= %s', index_child, device_name)
                status_item.setText(0, headline)
                #status_item.setText(0, device_name)
                #status_item.setText(1, child_diagnostic_status.message)
                rospy.logdebug(' StatusItem update 44')
            elif len(self.strip_child(name).split('/')) <= 2:
                status_item = StatusItem(child_diagnostic_status)
                status_item.update_children(child_diagnostic_status, diag_array)  # Recursive call.
                rospy.logdebug(' StatusItem update 55')
                status_item.setText(0, headline)
                #status_item.setText(0, device_name)
                #status_item.setText(1, child_diagnostic_status.message)
                rospy.logdebug(' StatusItem update 66')
                self._children_statusitems.append(status_item)
                rospy.logdebug(' StatusItem update 77')
                # new_statusitems.append(status_item)
                self.addChild(status_item)             
                rospy.logdebug(' StatusItem update 88')   
                      
        # self.addChildren(new_statusitems)  # QTreeWidgetItem::addChildren(QList)
        # return set/dict of error&warn
        
        rospy.logdebug(' ------ Statusitem.update_children err=%d warn=%d', errors, warnings)
        return { 'times_errors' : errors, 'times_warnings' : warnings }

    def on_click(self):
        if not self.inspector:
            rospy.logdebug('StatusItem.on_click No InspectorWindow Found')
            self.inspector = InspectorWindow(self.status)                        
            # self.connect(self.inspector.closeEvent) ## Doesn't work. Super doesn't have connect()
            self.inspector._close_window.connect(self.close_inspector_window)            
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
    Because Isaac failed to find a way to call a destructor of a class in python in general, he made this function, 
     intending it to be called by its parent object (in this case RobotMonitorWidget's instance) every timeline when a certain node gets removed.
    
    @author: Isaac Saito
    '''        
    def close(self):
        if self.inspector:
            # del self.inspector # Doesn't _close the window
            self.inspector.close()
            
        # _close children.
        for status_item in self._children_statusitems:
            status_item.close()
            
class RobotMonitorWidget(QWidget):
    # sig_err = Signal(str, int)
    # sig_warn = Signal(str, str, int)    
    sig_clear = Signal()

    def __init__(self, context, topic):
        super(RobotMonitorWidget, self).__init__()
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)),
                               'rqt_robot_monitor_mainwidget.ui')
        loadUi(ui_file, self)
        
        obj_name = 'Robot Monitor'
        self.setObjectName(obj_name)
        
        self.toplv_statusitems = []  # StatusItem
        self.warn_statusitems = []  # StatusItem. Contains ALL DEGREES (device top level, device' sub) in parallel. 
        self.err_statusitems = []  # StatusItem
        
        self.sig_clear.connect(self._clear)
        
        self.tree_all_devices.itemDoubleClicked.connect(self._tree_clicked)
        self.warn_tree.itemDoubleClicked.connect(self._tree_clicked)
        self.err_tree.itemDoubleClicked.connect(self._tree_clicked)
        
        self.tree_all_devices.resizeColumnToContents(0)
        self.tree_all_devices.resizeColumnToContents(1)
        
        # TODO Declaring timeline pane. Needs to be stashed away into .ui file but so far failed. 
        self.timeline_pane = TimelinePane(self)
        # self.timeline_pane.setMinimumSize(100, 50)
        #self.splitter.insertWidget(3, self.timeline_pane)
        self.vlayout_top.addWidget(self.timeline_pane)                

        self.paused = False

        self.topic = topic
        self.sub = rospy.Subscriber(
                                    self.topic,  # name of the topic 
                                    DiagnosticArray,  # type of the topic
                                    self._cb)
        self.setWindowTitle('Robot Monitor')        
        self.num_diag_msg_received = 0  # For debug
        
    def _cb(self, msg):        
        if not self.paused:            
            # self.sig_clear.emit()
            rospy.logdebug('_cb 00')
            self._update_devices_tree(msg)
            self._update_warns_errors(msg)
            rospy.logdebug('_cb 22')
            self.timeline_pane.add_message(msg)            
        
        self.num_diag_msg_received += 1
        rospy.logdebug('  RobotMonitorWidget _cb #%d', self.num_diag_msg_received)
        
    '''
    @param item: QTreeWidgetItem 
    @param column: int 
    '''
    def _tree_clicked(self, item, column):
        rospy.logdebug('RobotMonitorWidget _tree_clicked col=%d', column)  # Shown.
        item.on_click()
        
    '''
    Update the tree from the bottom    
    @param diag_array: DiagnosticArray 

    @todo: 11/5/2012 Currently, in case some devices disappear while running this program, there's no way to remove those from the device-tree. 
    '''
    def _update_devices_tree(self, diag_array):
        statusnames_curr_toplevel = [get_nice_name(k.name) for k in self.toplv_statusitems]  # Only the k variable that pops up at the end is processed by get_nice_name.
        # new_statusitems_toplv = []
        
        for diagnostic_status_new in self._get_toplv_diagnosticstatus_from_new_msg(diag_array):
            name = get_nice_name(diagnostic_status_new.name)
            rospy.logdebug('_update_devices_tree 0 name @ toplevel %s', name)
            dict_status = 0
            if name in statusnames_curr_toplevel:  # No change of names in toplevel since last time. 
                statusitem = self.toplv_statusitems[statusnames_curr_toplevel.index(name)]
                
                # TODO 10/30/2012/Isaac/Next lines (where status update_children occurs) can cause flickering as the number of nodes increase.
                #                       Can refer to original robot_model for an idea.
                dict_status = statusitem.update_children(diagnostic_status_new, diag_array)
                times_errors = dict_status['times_errors']
                times_warnings = dict_status['times_warnings']
                update_status_images(diagnostic_status_new, statusitem)
                                                
                # TODO Update status text on each node using dict_status.
                base_text = gen_headline_status_green(statusitem.status)
                # errwarn_text = gen_headline_warn_or_err(statusitem.status)
 
                rospy.logdebug('OOOOO _update_devices_tree warn_id= %s\n\t\t\tdiagnostic_status.name = %s\n\t\t\t\tNormal status diag_array = %s',
                               statusitem.warning_id, diagnostic_status_new.name, base_text)
                
                if (times_errors > 0 or times_warnings > 0):
                    #base_text = "(Err: %s, Wrn: %s) %s" % (times_errors, times_warnings, get_nice_name(diagnostic_status_new.name))
                    base_text = "(Err: %s, Wrn: %s) %s %s" % (times_errors, times_warnings, get_nice_name(diagnostic_status_new.name), diagnostic_status_new.message)
                    rospy.logdebug('_update_dev_tree 111 text to show=%s', base_text)
                    statusitem.setText(0, base_text)
                    #statusitem.setText(1, diagnostic_status_new.message)
                else:
                    rospy.logdebug('_update_dev_tree 222 text to show=%s', base_text)
                    statusitem.setText(0, base_text)
                    #statusitem.setText(1, '-')
                    
#                if (times_errors > 0 and statusitem.error_id is not None):
#                    statusitem.setText(0, errwarn_text)
#                    rospy.logdebug(' WARN _update_devices_tree warn 44')
#                elif (times_warnings > 0 and statusitem.warning_id is not None):
#                    statusitem.setText(0, errwarn_text)
#                    rospy.logdebug(' WARN _update_devices_tree 0 warn txt= %s', errwarn_text)
#                    rospy.logdebug(' WARN _update_devices_tree warn 55')
            else:
                rospy.logdebug('_update_dev_tree 333')
                new_status_item = StatusItem(diagnostic_status_new)
     
                # TODO receive return value set and use them.
                rospy.logdebug('_update_dev_tree 444')
                new_status_item.update_children(diagnostic_status_new, diag_array)
                
                # TODO Figure out if a statusitem and its subtree contains errors.
                # new_status_item.setIcon(0, self._error_icon) # This shows NG icon at the beginning of each statusitem.
                rospy.logdebug('_update_dev_tree 555')
                update_status_images(diagnostic_status_new, new_status_item)
                
                rospy.logdebug('_update_dev_tree 666')
                self.toplv_statusitems.append(new_status_item)
                
                # TODO new_statusitems_toplv might not be necessary - toplv_statusitems might substitute.
                rospy.logdebug('_update_dev_tree 777')
                # new_statusitems_toplv.append(new_status_item)
                rospy.logdebug(' _update_devices_tree 2 diagnostic_status_new.name %s', new_status_item.name)
                self.tree_all_devices.addTopLevelItem(new_status_item)
        rospy.logdebug('_update_dev_tree 888')
        # self.tree_all_devices.addTopLevelItems(new_statusitems_toplv)
        rospy.logdebug('_update_dev_tree 999')
        #self.tree_all_devices.sortItems (0, Qt.AscendingOrder)
        rospy.logdebug('_update_dev_tree 10-10-10')
        
    '''
    Return an array that contains DiagnosticStatus only at the top level of the given msg.  
    @param msg: DiagnosticArray 
    @return array of 'status' (ie. array of DiagnosticStatus[])
    '''
    def _get_toplv_diagnosticstatus_from_new_msg(self, diag_array):
        ret = []
        for diagnostic_status in diag_array.status:
            if len(diagnostic_status.name.split('/')) == 2:
                rospy.logdebug(" _get_toplv_diagnosticstatus_from_new_msg TOP lev %s ", diagnostic_status.name)
                ret.append(diagnostic_status)        
            else:
                rospy.logdebug(" _get_toplv_diagnosticstatus_from_new_msg Not top lev %s ", diagnostic_status.name)
        return ret

    '''
    @param key: string
    @param statusitems: DiagnosticStatus[]  
    @return: int of index that key is found in array. -1 if not found
    '''
    def _contains(self, key, statusitems):
        names = [get_nice_name(k.name) for k in statusitems]
        
        # debug
        for elem in names:
            rospy.logdebug('** _contains key= %s', elem)
        
        if key in names:
            rospy.logdebug('** _contains key IS contained.')
            return names.index(key)
        else:
            rospy.logdebug('** _contains key IS NOT contained.')
            return -1 
        
    """
    Update the warning and error trees. 
    
    Unlike _update_devices_tree function where all DiagnosticStatus need to be inspected constantly, 
    this function is used in a trial to reduce unnecessary inspection of the status level for all DiagnosticStatus contained in the incoming DiagnosticArray msg.
    
    @param msg: DiagnosticArray 
    """
    def _update_warns_errors(self, diag_array):
        #self._update_flat_tree(diag_array, DiagnosticStatus.WARN, self.warn_statusitems, self.warn_tree)
        self._update_flat_tree(diag_array)
        rospy.logdebug('_update_warns_errors 11')
        #self._update_flat_tree(diag_array, DiagnosticStatus.ERROR, self.err_statusitems, self.err_tree)
        rospy.logdebug('_update_warns_errors 22')
       
    def pause(self, msg):
        self.paused = True
        # self.sig_clear.emit()
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
    Update the given flat tree (that doesn't show children. All elements are top level) with all the DiagnosticStatus instances contained in the given DiagnosticArray, 
    regardless of the degree of the device.
    
    @param diag_arr: DiagnosticArray class.
    @param diagstat_lev: int. Intended either DiagnosticStatus.{ WARN, ERROR }.
    @param statitems_existing: StatusItem[] that are shown on QTreeWidget at this instant. 
    @param itemtree: QTreeWidget.
    @author: Isaac Saito
    '''
    def _update_flat_tree(self, diag_arr):
        # all_lev_statitems_tobe_shown = []
        statusnames_curr_toplevel = [get_nice_name(k.name) for k in self.toplv_statusitems]
        for diag_stat_new in diag_arr.status:
            # Children of toplevel items are taken care of, 
            #  by examining all DiagnosticStatus array elements in DiagnosticArray.
            level = diag_stat_new.level
            dev_name = get_nice_name(diag_stat_new.name)            
            if dev_name in statusnames_curr_toplevel:
                continue  # Skipping top level device to be shown (all device tree always shows it, so no need to show them on warn / err trees). 
            
            statitems_existing = []
            itemtree = ''
            target_lev = ''
            if level == DiagnosticStatus.WARN:
                statitems_existing = self.warn_statusitems
                itemtree = self.warn_tree
                target_lev = DiagnosticStatus.WARN
            elif level == DiagnosticStatus.ERROR:
                statitems_existing = self.err_statusitems
                itemtree = self.err_tree
                target_lev = DiagnosticStatus.ERROR
                
            dev_index = self._contains(dev_name, statitems_existing)                
            
            #rospy.logdebug('** _update_flat_tree dev_index=%s len of warn_statusitems=%d', dev_index, len(statitems_existing))
            headline = "%s : %s" % (diag_stat_new.name, diag_stat_new.message)
            
            if 0 <= dev_index:  # Not a new device for warn tree. 
                statitem_existing = statitems_existing[dev_index]
                
#                item_id = {
#                    DiagnosticStatus.WARN: lambda: statitem_existing.warning_id,
#                    DiagnosticStatus.ERROR: lambda: statitem_existing.error_id,
#                }[diagstat_lev]()
                #rospy.logdebug('** _update_flat_tree statusitem.lev=%s item_id=%s name=%s', level, item_id, statitem_existing.name)
                
                if (DiagnosticStatus.WARN != level and statitem_existing.warning_id is not None):
                    rospy.logdebug('_update_warning_tree REMOVE FROM TREE name=%s', statitem_existing.name)
                    # self.warn_tree.removeItemWidget(statitem_existing, 0) # removeItemWidget doesn't remove an item from tree. 
                    self.warn_tree.takeTopLevelItem(self.warn_tree.indexOfTopLevelItem(statitem_existing))  # (statitem_existing, 0)
                    rospy.logdebug('_update_flat_tree REMOVE 22')
                    # statitems_existing.remove(statitem_existing) #pyside causes error here (NotImplementedError: operator not implemented.)
                    self.warn_statusitems.pop(dev_index)
                    # del statitem_existing #Trying to delete in nested clause will cause terrible error.
                    rospy.logdebug('_update_flat_tree REMOVE FROM TREE Done')
                elif (DiagnosticStatus.WARN == level and statitem_existing.warning_id is not None):
                    rospy.logdebug('** _update_flat_tree statusitem. 11 NO CHANGE.')
                elif (DiagnosticStatus.WARN == level and statitem_existing.warning_id is None):  # Doesn't need to do anything.
                    rospy.logdebug('** _update_flat_tree statusitem. 22')
                
                ## Remove ERROR    
                elif (DiagnosticStatus.ERROR != level and statitem_existing.error_id is not None):
                    rospy.logdebug(' err REMOVE FROM TREE name=%s', statitem_existing.name)
                    # self.warn_tree.removeItemWidget(statitem_existing, 0) # removeItemWidget doesn't remove an item from tree. 
                    self.err_tree.takeTopLevelItem(self.err_statusitems.indexOfTopLevelItem(statitem_existing))  # (statitem_existing, 0)
                    rospy.logdebug(' err REMOVE 22')
                    # statitems_existing.remove(statitem_existing) #pyside causes error here (NotImplementedError: operator not implemented.)
                    self.err_statusitems.pop(dev_index)
                    # del statitem_existing #Trying to delete in nested clause will cause terrible error.
                    rospy.logdebug(' err REMOVE FROM TREE Done')
                elif (DiagnosticStatus.ERROR == level and statitem_existing.error_id is not None):
                    rospy.logdebug('** _update_flat_tree statusitem. 11 NO CHANGE.')
                elif (DiagnosticStatus.ERROR == level and statitem_existing.error_id is None):  # Doesn't need to do anything.
                    rospy.logdebug('** err statusitem. 22')
                     
                else:
                    rospy.logdebug('** _update_flat_tree statusitem. ELSE ')
                           
            elif dev_index < 0 and 0 < level:
                # New device for warn tree. Create new statusitem instance.
                statitem_new = StatusItem(diag_stat_new)
                if DiagnosticStatus.WARN == level:
                    statitem_new.warning_id = random.random()
                elif DiagnosticStatus.ERROR == level:
                    statitem_new.error_id = random.random()
                # statitem_new.setText(0, headline)
                rospy.logdebug(' NEW NEW NEW 11 _update_warning_tree diag_stat_new.name= %s, diag_stat_new.msg= %s',
                              diag_stat_new.name, diag_stat_new.message)
                statitem_new.setText(0, headline)
                #statitem_new.setText(1, diag_stat_new.message)
                statitem_new.setIcon(0, image_dict[level])
                rospy.logdebug(' NEW NEW NEW 22 _update_warning_tree')
                # all_lev_statitems_tobe_shown.append(statitem_new)                
                statitems_existing.append(statitem_new)                
                rospy.logdebug(' NEW NEW NEW 22_11 _update_warning_tree')
                itemtree.addTopLevelItem(statitem_new)
        rospy.logdebug(' NEW NEW NEW 33_00 _update_warning_tree')
        # itemtree.addTopLevelItems(all_lev_statitems_tobe_shown)                
        rospy.logdebug(' NEW NEW NEW 33_11 _update_flat_tree')
        #itemtree.sortItems (0, Qt.AscendingOrder)
        rospy.logdebug(' NEW NEW NEW 33 _update_flat_tree')
#    '''
#    Update the given flat tree (that doesn't show children. All elements are top level) with all the DiagnosticStatus instances contained in the given DiagnosticArray, 
#    regardless of the degree of the device.
#    
#    @param diag_arr: DiagnosticArray class.
#    @param diagstat_lev: int. Intended either DiagnosticStatus.{ WARN, ERROR }.
#    @param statitems_existing: StatusItem[] that are shown on QTreeWidget at this instant. 
#    @param itemtree: QTreeWidget.
#    @author: Isaac Saito
#    '''
#    def _update_flat_tree(self, diag_arr, diagstat_lev, statitems_existing, itemtree):
#        # all_lev_statitems_tobe_shown = []
#        statusnames_curr_toplevel = [get_nice_name(k.name) for k in self.toplv_statusitems]
#        for diag_stat_new in diag_arr.status:
#            # Children of toplevel items are taken care of, 
#            #  by examining all DiagnosticStatus array elements in DiagnosticArray.
#            
#            dev_name = get_nice_name(diag_stat_new.name)            
#            if dev_name in statusnames_curr_toplevel:
#                continue  # Skipping top level device to be shown (all device tree always shows it, so no need to show them on warn / err trees). 
#            
#            dev_index = self._contains(dev_name, statitems_existing)
#            rospy.logdebug('** _update_flat_tree dev_index=%s len of warn_statusitems=%d',
#                          dev_index, len(statitems_existing))
#            headline = "%s : %s" % (diag_stat_new.name, diag_stat_new.message)
#            level = diag_stat_new.level
#            if 0 <= dev_index:  # Not a new device for warn tree. 
#                statitem_existing = statitems_existing[dev_index]
#                
#                item_id = {
#                    DiagnosticStatus.WARN: lambda: statitem_existing.warning_id,
#                    DiagnosticStatus.ERROR: lambda: statitem_existing.error_id,
#                }[diagstat_lev]()
#                rospy.logdebug('** _update_flat_tree statusitem.lev=%s item_id=%s name=%s',
#                          level, item_id, statitem_existing.name)
#                
#                if (diagstat_lev != level and item_id is not None):
#                    rospy.logdebug('_update_warning_tree REMOVE FROM TREE name=%s', statitem_existing.name)
#                    # self.warn_tree.removeItemWidget(statitem_existing, 0) # removeItemWidget doesn't remove an item from tree. 
#                    itemtree.takeTopLevelItem(itemtree.indexOfTopLevelItem(statitem_existing))  # (statitem_existing, 0)
#                    rospy.logdebug('_update_flat_tree REMOVE 22')
#                    # statitems_existing.remove(statitem_existing) #pyside causes error here (NotImplementedError: operator not implemented.)
#                    statitems_existing.pop(dev_index)
#                    # del statitem_existing #Trying to delete in nested clause will cause terrible error.
#                    rospy.logdebug('_update_flat_tree REMOVE FROM TREE Done')
#                elif (diagstat_lev == level and item_id is not None):
#                    rospy.logdebug('** _update_flat_tree statusitem. 11 NO CHANGE.')
#                elif (diagstat_lev == level and item_id is None):  # Doesn't need to do anything.
#                    rospy.logdebug('** _update_flat_tree statusitem. 22') 
#                else:
#                    rospy.logdebug('** _update_flat_tree statusitem. ELSE ')
#                           
#            elif dev_index < 0 and diagstat_lev == diag_stat_new.level:
#                # New device for warn tree. Create new statusitem instance.
#                statitem_new = StatusItem(diag_stat_new)
#                if DiagnosticStatus.WARN == diagstat_lev:
#                    statitem_new.warning_id = random.random()
#                elif DiagnosticStatus.ERROR == diagstat_lev:
#                    statitem_new.error_id = random.random()
#                # statitem_new.setText(0, headline)
#                rospy.logdebug(' NEW NEW NEW 11 _update_warning_tree diag_stat_new.name= %s, diag_stat_new.msg= %s',
#                              diag_stat_new.name, diag_stat_new.message)
#                statitem_new.setText(0, headline)
#                #statitem_new.setText(1, diag_stat_new.message)
#                statitem_new.setIcon(0, image_dict[level])
#                rospy.logdebug(' NEW NEW NEW 22 _update_warning_tree')
#                # all_lev_statitems_tobe_shown.append(statitem_new)                
#                statitems_existing.append(statitem_new)                
#                rospy.logdebug(' NEW NEW NEW 22_11 _update_warning_tree')
#                itemtree.addTopLevelItem(statitem_new)
#        rospy.logdebug(' NEW NEW NEW 33_00 _update_warning_tree')
#        # itemtree.addTopLevelItems(all_lev_statitems_tobe_shown)                
#        rospy.logdebug(' NEW NEW NEW 33_11 _update_flat_tree')
#        #itemtree.sortItems (0, Qt.AscendingOrder)
#        rospy.logdebug(' NEW NEW NEW 33 _update_flat_tree')
            
    def _close(self):  # 10/24/Isaac/When this is called?
        rospy.logdebug('RobotMonitorWidget in _close')
        if self.sub:
            self.sub.unregister()
            self.sub = None

    def _shutdown(self):
        rospy.logdebug('RobotMonitorWidget in _shutdown')
        # Close all StatusItem (and each associated InspectWidget)        
        # self.tree_all_devices.clear()  # Doesn't work for the purpose (inspector windows don't get closed)
        
        # TODO stop timers
        
        # TODO stop publishers
        # TODO unsubscribe from Topics
                 
        for item in self.err_statusitems:
            item.close()
        for item in self.warn_statusitems:
            item.close()
        for item in self.toplv_statusitems:
            item.close()
