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

import roslib;roslib.load_manifest('rqt_robot_monitor')
import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, Signal
from python_qt_binding.QtGui import QIcon, QTextEdit, QWidget

from .abst_status_widget import AbstractStatusWidget
from .chronologic_state import InstantaneousState, StatusItem
from .inspector_window import InspectorWindow
from .time_pane import TimelinePane
from .util_robot_monitor import Util

class RobotMonitorWidget(AbstractStatusWidget):
    """
    shutdown function needs to be called when the class terminates.
    
    RobotMonitorWidget itself doesn't store previous states. It instead
     delegates that function to TimelinePane class. 
    """
    
    _sig_clear = Signal()    
    _sig_tree_nodes_updated = Signal(int)
    _TREE_ALL = 1
    _TREE_WARN = 2
    _TREE_ERR = 3

    '''
    @param context:
    @param topic:  
    '''
    def __init__(self, context, topic):
        super(RobotMonitorWidget, self).__init__()
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)),
                               'rqt_robot_monitor_mainwidget.ui')
        loadUi(ui_file, self)
        
        obj_name = 'Robot Monitor'
        self.setObjectName(obj_name)
        self.setWindowTitle(obj_name)        
        
        self._toplv_statusitems = []  # StatusItem
        self._warn_statusitems = []  # StatusItem. Contains ALL DEGREES 
                                 # (device top level, device' _sub) in parallel. 
        self._err_statusitems = []  # StatusItem

        self.tree_all_devices.itemDoubleClicked.connect(self._tree_clicked)
        self.warn_tree.itemDoubleClicked.connect(self._tree_clicked)
        self.err_tree.itemDoubleClicked.connect(self._tree_clicked)
        
        self.tree_all_devices.resizeColumnToContents(0)
        
        self._sig_clear.connect(self._clear)
        self._sig_tree_nodes_updated.connect(self._tree_nodes_updated)

        # self.tree_all_devices.sortByColumn(0, Qt.AscendingOrder) 
        ##      No effect. Sorting didn't get enabled.
        
        # TODO Declaring timeline pane. 
        #      Needs to be stashed away into .ui file but so far failed. 
        self.timeline_pane = TimelinePane(self, Util._SECONDS_TIMELINE, 
                                          self._cb, 
                                          self._get_color_for_value, 
                                          self._on_pause)
        self.vlayout_top.addWidget(self.timeline_pane)
        self.timeline_pane.show()                

        self.paused = False
        self._is_stale = False
        self.last_message_time = 0.0
        
        self._timer = QTimer()
        # self._timer.timerEvent.connect(self._update_message_state)
        self._timer.timeout.connect(self._update_message_state)
        self._timer.start(1000);

        self.num_diag_msg_received = 0  # For debug
        
        self.topic = topic
        self._sub = rospy.Subscriber(
                                    self.topic,  # name of the topic 
                                    DiagnosticArray,  # type of the topic
                                    self._cb)        
        
    def _cb(self, msg, is_forced = False):
        """
        @param msg: DiagnosticArray
        @author: Isaac Saito
        """
        if not self.paused and not is_forced:
            self.timeline_pane._new_diagnostic(msg)
            self._update_devices_tree(msg)       
            self._update_warns_errors(msg)
            self.on_new_message_received(msg)            
            rospy.logdebug('  RobotMonitorWidget _cb stamp=%s',
                       msg.header.stamp)
        elif is_forced:
            self._update_devices_tree(msg)       
            self._update_warns_errors(msg)
        
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
        
    def _update_devices_tree(self, diag_array):
        """
        Update the tree from the bottom
        @param diag_array: DiagnosticArray
        
        @todo: 11/5/2012 Currently, in case some devices disappear 
                     while running this program, there's no way to remove 
                     those from the device-tree.
        """
         
        statusnames_curr_toplevel = [Util.get_nice_name(k.name) 
                                     for k in self._toplv_statusitems]  
        # Only the k variable that pops up at the end is 
        # processed by Util.get_nice_name.
        
        for diagnostic_status_new in self._get_toplv_diagnosticstatus_from_new_msg(diag_array):
            name = Util.get_nice_name(diagnostic_status_new.name)
            rospy.logdebug('_update_devices_tree 0 name @ toplevel %s', name)
            dict_status = 0
            if name in statusnames_curr_toplevel:  # No change of names 
                                                 # in toplevel since last time. 
                statusitem = self._toplv_statusitems[
                                        statusnames_curr_toplevel.index(name)]

                dict_status = statusitem.update_children(diagnostic_status_new,
                                                         diag_array)
                times_errors = dict_status['times_errors']
                times_warnings = dict_status['times_warnings']
                Util._update_status_images(diagnostic_status_new, statusitem)
                                                
                # TODO Update status text on each node using dict_status.
                base_text = Util.gen_headline_status_green(statusitem.status)
                # errwarn_text = Util.gen_headline_warn_or_err(statusitem.status)
 
                rospy.logdebug('_update_devices_tree warn_id= %s\n\t\t\t' + 
                               'diagnostic_status.name = %s\n\t\t\t\t' + 
                               'Normal status diag_array = %s',
                               statusitem.warning_id,
                               diagnostic_status_new.name, base_text)
                
                if (times_errors > 0 or times_warnings > 0):                    
                    base_text = "(Err: %s, Wrn: %s) %s %s" % (
                                   times_errors,
                                   times_warnings,
                                   Util.get_nice_name(diagnostic_status_new.name),
                                   diagnostic_status_new.message)
                    rospy.logdebug('_update_dev_tree 1 text to show=%s',
                                   base_text)
                    statusitem.setText(0, base_text)
                    statusitem.setText(1, diagnostic_status_new.message)
                else:
                    rospy.logdebug('_update_dev_tree 2 text to show=%s',
                                   base_text)
                    statusitem.setText(0, base_text)
                    statusitem.setText(1, 'OK')
       
            else:
                new_status_item = StatusItem(diagnostic_status_new)
     
                # TODO receive return value set and use them.
                new_status_item.update_children(diagnostic_status_new,
                                                diag_array)
                
                # TODO Figure out if a statusitem and 
                #      its subtree contains errors.
                # new_status_item.setIcon(0, self._error_icon) 
                #      This shows NG icon at the beginning of each statusitem.
                Util._update_status_images(diagnostic_status_new,
                                           new_status_item)
                
                self._toplv_statusitems.append(new_status_item)
                
                # TODO new_statusitems_toplv might not be necessary - 
                #                        _toplv_statusitems might substitute.
                # new_statusitems_toplv.append(new_status_item)
                rospy.logdebug(' _update_devices_tree 2 ' + 
                               'diagnostic_status_new.name %s',
                               new_status_item.name)
                self.tree_all_devices.addTopLevelItem(new_status_item)

        self._sig_tree_nodes_updated.emit(self._TREE_ALL)
    
    def _tree_nodes_updated(self, tree_type):
        tree_obj = None
        if self._TREE_ALL == tree_type:
            tree_obj = self.tree_all_devices
        elif self._TREE_WARN == tree_type:
            tree_obj = self.warn_tree       
        if self._TREE_ERR == tree_type:
            tree_obj = self.err_tree
        tree_obj.resizeColumnToContents(0)
            
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
    TODO Needs renamed to better describing one.
    
    @param key: string
    @param statusitems: DiagnosticStatus[]  
    @return: int of index that key is found in array. -1 if not found
    '''
    def _contains(self, key, statusitems):
        names = [Util.get_nice_name(k.name) for k in statusitems]
        
        rospy.logdebug('\t_contains len of names=%d statusitems=%d',
                       len(names), len(statusitems))
#        for name in names:  # This loop is only for debug 
#            rospy.loginfo('\t_contains Required key=%s CONTAINED KEY= %s', 
#                          key, name)
        
        if key in names:
            rospy.logdebug(' _contains key IS contained.')
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
        self._update_flat_tree(diag_array, self._toplv_statusitems)
        # self._update_flat_tree(diag_array)
       
    def _pause(self, msg):
        """
        Ignored if already being paused.
        
        @param msg: DiagnosticArray 
        """
        
        if not self.paused:
            self.paused = True
            self._cb(msg)
                            
    def _unpause(self):
        self.paused = False

    def _on_pause(self, paused, diagnostic_arr):
        """
        Check if InspectorWindows are set. If they are, pause them.
        
        Pay attention not to confuse with _pause func.
        
        @param paused: bool
        @param diagnostic_arr: 
         
        Copied from robot_monitor.
        """
        
        if paused:
            self._pause(diagnostic_arr)
        # if (not paused and len(self._viewers) > 0):
        elif (len(self._toplv_statusitems) > 0):
            diag_array_queue = self.timeline_pane._get_diagnosticarray()
            statitems = []
            for diag_arr in diag_array_queue:
                state_instant = InstantaneousState()
                state_instant.update(diag_arr)
                statitems.append(state_instant)
        
        for statitem_toplv in self._toplv_statusitems:
            if (paused):
                statitem_toplv._disable()
            else:
                statitem_toplv._enable()    
                for state_instant in statitems:
                    all = state_instant.get_items()
                    if (all.has_key(statitem_toplv.get_name())):
                        statitem_toplv.update(
                                        all[statitem_toplv.get_name()].status)
                        
    '''
    11/5/2012/Isaac/Seems not really kicked from anywhere.
    '''
    def _clear(self):
        rospy.logdebug(' RobotMonitorWidget _clear called ')
        self.err_tree.clear()
        self.warn_tree.clear()

    def _update_flat_tree(self, diag_arr, statusitems_curr_toplevel):
        """
        
        Update the given flat tree (ie. tree that doesn't show children nodes - 
        all of its elements will be shown on top level) with 
        all the DiagnosticStatus instances contained in the given DiagnosticArray, 
        regardless of the level of the device in a device category.
    
        @param diag_arr: a DiagnosticArray instance.
        @param statusitems_curr_toplevel: list of StatusItem.
    
        @author: Isaac Saito
        """
        
        for diag_stat_new in diag_arr.status:
            stat_lv_new = diag_stat_new.level
            dev_name = diag_stat_new.name
            dev_index_warn_curr = self._contains(Util.get_nice_name(dev_name),
                                                 self._warn_statusitems)
            dev_index_err_curr = self._contains(Util.get_nice_name(dev_name),
                                                self._err_statusitems)            
            headline = "%s" % diag_stat_new.name
            rospy.logdebug('###_update_flat_tree index warn= %d err= %d %s',
                           dev_index_warn_curr, dev_index_err_curr, headline)
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
                    self.add_statitem(statitem_curr, self._warn_statusitems,
                                      self.warn_tree, headline, 
                                      diag_stat_new.message, stat_lv_new)
                elif (dev_index_warn_curr < 0 and dev_index_err_curr < 0):
                    statitem_new = StatusItem(diag_stat_new)
                    self.add_statitem(statitem_new, self._warn_statusitems,
                                       self.warn_tree, headline,
                                       diag_stat_new.message, stat_lv_new)
                    self._warn_statusitems.append(statitem_new)
            elif ((DiagnosticStatus.ERROR == stat_lv_new) or
                  (DiagnosticStatus.STALE == stat_lv_new)):
                if 0 <= dev_index_warn_curr:
                    statitem_curr = self._remove_statitem(dev_index_warn_curr,
                                                          self._warn_statusitems,
                                                          self.warn_tree)
                    self.add_statitem(statitem_curr, self._err_statusitems,
                                      self.err_tree, headline,
                                      diag_stat_new.message, stat_lv_new)
                elif (dev_index_warn_curr < 0 and dev_index_err_curr < 0):
                    statitem_new = StatusItem(diag_stat_new)
                    self.add_statitem(statitem_new, self._err_statusitems,
                                      self.err_tree, headline,
                                      diag_stat_new.message, stat_lv_new)
        
        self._sig_tree_nodes_updated.emit(self._TREE_WARN)
        self._sig_tree_nodes_updated.emit(self._TREE_ERR)
        
    '''
    @author: Isaac Saito
    '''
    def add_statitem(self, statusitem, statitem_list,
                      tree, headline, statusmsg, statlevel):
        
        if 'Warning' == statusmsg or 'Error' == statusmsg:
            return 
        
        statusitem.setText(0, headline)
        statusitem.setText(1, statusmsg)
        statusitem.setIcon(0, Util._IMG_DICT[statlevel])
        statitem_list.append(statusitem)                
        tree.addTopLevelItem(statusitem)
        rospy.logdebug(' add_statitem statitem_list length=%d',
                       len(statitem_list))
           
    '''
    @author: Isaac Saito
    '''            
    def _remove_statitem(self, item_index, item_list, tree):
        statitem_existing = item_list[item_index]
        tree.takeTopLevelItem(tree.indexOfTopLevelItem(statitem_existing))
        item_list.pop(item_index)
        return statitem_existing

    def on_new_message_received(self, msg):
        """
        Copied from robot_monitor
    
        \brief Called whenever a new message is received by the timeline.  
        Different from new_message in that it
        is called even if the timeline is paused, and only when a new message is 
        received, not when the timeline is scrubbed
        """    
        self.last_message_time = rospy.get_time()

    '''
    @author: Isaac Saito (ported from robot_monitor)
    '''
    def _update_message_state(self):
        current_time = rospy.get_time()
        time_diff = current_time - self.last_message_time
        rospy.logdebug('_update_message_state time_diff= %s ' + 
                       'self.last_message_time=%s', time_diff,
                       self.last_message_time)
        if (time_diff > 10.0):
            self.timeline_pane._msg_label.setText("Last message received " + 
                                               "%s seconds ago"
                                               % (int(time_diff)))
            self._is_stale = True
        else:
            seconds_string = "seconds"
            if (int(time_diff) == 1):
                seconds_string = "second"
            self.timeline_pane._msg_label.setText(
                 "Last message received %s %s ago" % (int(time_diff),
                                                      seconds_string))
            self._is_stale = False        
            
    # 11/19/2012/Isaac _close will be deleted.        
#    def _close(self):  # 10/24/Isaac/When this is called?
#        rospy.logdebug('RobotMonitorWidget in _close')
#        if self._sub:
#            self._sub.unregister()
#            self._sub = None

    '''
    This needs to be called whenever this class terminates.
    '''
    def _shutdown(self):
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

        # stop timers 
        self._timer.stop()
        del self._timer

    def _get_color_for_value(self, queue_diagnostic, color_index):
        """
        Taken from robot_monitor
        
        Overridden.
        
        @param color_index: int 
        """
        len_q = len(queue_diagnostic)
        rospy.logdebug(' _get_color_for_value color_index=%d len_q=%d',
                      color_index, len_q)
        # if (color_index == 1 and len_q == 0): #TODO Needs to be reverted back.
        if (color_index <= 2 and len_q == 0):  # TODO This line is Debug only
            return QColor('grey')
        return Util._get_color_for_message(queue_diagnostic[color_index - 1])
                               # When _queue_diagnostic is empty,
                               # this yield error when color_index > 0.   
                

    def _save_settings(self, plugin_settings, instance_settings):
        instance_settings.set_value('splitter', self.splitter.saveState())

    def _restore_settings(self, plugin_settings, instance_settings):
        if instance_settings.contains('splitter'):
            self.splitter.restoreState(instance_settings.value('splitter'))
        else:
            self.splitter.setSizes([100, 100, 200])

