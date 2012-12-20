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
import rospkg

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, Signal
from python_qt_binding.QtGui import QIcon, QTextEdit, QWidget
import rospy

from .abst_status_widget import AbstractStatusWidget
from .chronologic_state import InstantaneousState, StatusItem
from .time_pane import TimelinePane
from .util_robot_monitor import Util

class RobotMonitorWidget(AbstractStatusWidget):
    """
    NOTE: RobotMonitorWidget.shutdown function needs to be called 
          when the instance of this class terminates.
    
    RobotMonitorWidget itself doesn't store previous diagnostic states. 
    It instead delegates that function to TimelinePane class. 
    """
    
    _sig_tree_nodes_updated = Signal(int)
    _sig_new_diagnostic = Signal(DiagnosticArray)
    _TREE_ALL = 1
    _TREE_WARN = 2
    _TREE_ERR = 3

    def __init__(self, context, topic):
        """
        
        :type context:
        :type topic: 
        """
          
        super(RobotMonitorWidget, self).__init__()
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_robot_monitor'), 'resource', 'rqt_robot_monitor_mainwidget.ui')
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
        
        self._sig_tree_nodes_updated.connect(self._tree_nodes_updated)
        
        # TODO Declaring timeline pane. 
        #      Needs to be stashed away into .ui file but so far failed. 
        self.timeline_pane = TimelinePane(self, Util._SECONDS_TIMELINE, 
                                          #self._cb, 
                                          self.get_color_for_value, 
                                          self.on_pause)
        
        self.vlayout_top.addWidget(self.timeline_pane)
        self.timeline_pane.show()                

        self._paused = False
        self._is_stale = False
        self._last_message_time = 0.0
        
        self._timer = QTimer()
        # self._timer.timerEvent.connect(self._update_message_state)
        self._timer.timeout.connect(self._update_message_state)
        self._timer.start(1000);

        self.num_diag_msg_received = 0  # For debug
        
        self._sub = rospy.Subscriber(
                                    topic,  # name of the topic 
                                    DiagnosticArray,  # type of the topic
                                    self._cb)
        self._sig_new_diagnostic.connect(self.new_diag)
        
    def _cb(self, msg):
        """
        Intended to be called from non-Qt thread, 
        ie. ROS Subscriber in particular.
        
        :type msg: DiagnosticArray  
        """
        self._sig_new_diagnostic.emit(msg)
        
    def new_diag(self, msg, is_forced = False):        
        """
        When not paused, this public method updates all the treewidgets 
        contained in this class, and also notifies the StatusItem instances 
        that are stored in the all-device-tree, which eventually updates 
        the InspectorWindows in them.  
        
        :type msg: DiagnosticArray
        :param is_forced: Intended for non-incoming-msg trigger 
                          (in particular, from child object like TimelinePane). 
        @author: Isaac Saito
        """
        if not self._paused and not is_forced:
            self.timeline_pane.new_diagnostic(msg)
            self._update_devices_tree(msg)       
            self._update_warns_errors(msg)
            self._on_new_message_received(msg)
            
            self._notify_statitems(msg)
                        
            rospy.logdebug('  RobotMonitorWidget _cb stamp=%s',
                       msg.header.stamp)
        elif is_forced:
            self._update_devices_tree(msg)       
            self._update_warns_errors(msg)
        
        self.num_diag_msg_received += 1
        rospy.logdebug('  RobotMonitorWidget _cb #%d',
                       self.num_diag_msg_received)
    
    def _notify_statitems(self, diag_arr):
        """
        Notify new message arrival to all existing InespectorWindow 
        instances that are encapsulated in StatusItem instances contained 
        in self._toplv_statusitems.
        """
        
        for statitem_new in diag_arr.status:
            corresp = Util.get_correspondent(statitem_new.name,
                                             self._toplv_statusitems)
            statitem_prev = corresp[Util._DICTKEY_STATITEM] 
            if statitem_prev and statitem_prev.inspector:
                rospy.logdebug('  RobotMonitorWidget _notify_statitems ' +
                               'name=%s len toplv items=%d',
                               statitem_new.name, len(self._toplv_statusitems))  
                return
                       
    def resizeEvent(self, evt):
        """Overridden from QWidget"""
        rospy.logdebug('RobotMonitorWidget resizeEvent')
        self.timeline_pane.redraw()
                 
    def _tree_clicked(self, item, column):
        """
        Slot to QTreeWidget.itemDoubleClicked 
        
        :type item: QTreeWidgetItem
        :type column: int
        """
        rospy.logdebug('RobotMonitorWidget _tree_clicked col=%d', column) 
        item.on_click()
        
    def _update_devices_tree(self, diag_array):
        """
        Update the tree from the bottom
        
        :type diag_array: DiagnosticArray        
        """
        #TODO(Isaac) 11/5/2012 Currently, in case some devices disappear 
        #            while running this program, there's no way to remove 
        #            those from the device-tree.
         
        statusnames_curr_toplevel = [Util.get_nice_name(k.name) 
                                     for k in self._toplv_statusitems]  
        # Only the k variable that pops up at the end is 
        # processed by Util.get_nice_name.
        
        for diagnostic_status_new in self._get_toplv_diagnosticstatus_from_new_msg(
                                                                   diag_array):
            name = Util.get_nice_name(diagnostic_status_new.name)
            rospy.logdebug('_update_devices_tree 0 name @ toplevel %s', name)
            dict_status = 0
            if name in statusnames_curr_toplevel:  # No change of names 
                                                 # in toplevel since last time. 
                statusitem = self._toplv_statusitems[
                                        statusnames_curr_toplevel.index(name)]

                dict_status = statusitem.update_children(diagnostic_status_new,
                                                         diag_array)
                times_errors = dict_status[Util._DICTKEY_TIMES_ERROR]
                times_warnings = dict_status[Util._DICTKEY_TIMES_WARN]
                Util._update_status_images(diagnostic_status_new, statusitem)
                                                
                #TODO(Isaac) Update status text on each node using dict_status.
                base_text = Util.gen_headline_status_green(statusitem.status)
 
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
                
                # Figure out if a statusitem and its subtree contains errors.
                # new_status_item.setIcon(0, self._error_icon) 
                # This shows NG icon at the beginning of each statusitem.
                Util._update_status_images(diagnostic_status_new,
                                           new_status_item)
                
                self._toplv_statusitems.append(new_status_item)
                
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
            
    def _get_toplv_diagnosticstatus_from_new_msg(self, diag_array):
        """
        
        Return an array that contains DiagnosticStatus only at the top level of
        the given msg.
      
        :type msg: DiagnosticArray 
        :rtype: DiagnosticStatus[]
        """
        
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
 
    def _update_warns_errors(self, diag_array):
        """
        Update the warning and error trees.
            
        Unlike _update_devices_tree function where all DiagnosticStatus 
        need to be inspected constantly, this function is used in a trial 
        to reduce unnecessary inspection of the status level for all 
        DiagnosticStatus contained in the incoming DiagnosticArray msg.
            
        :type msg: DiagnosticArray
        """
            
        self._update_flat_tree(diag_array)
       
    def pause(self, msg):
        """
        Do nothing if already being _paused.
        
        :type msg: DiagnosticArray 
        """
        
        if not self._paused:
            self._paused = True
            self.new_diag(msg)
                            
    def unpause(self, msg = None):
        """
        :type msg: DiagnosticArray 
        """
        
        self._paused = False

    def on_pause(self, paused, diagnostic_arr):
        """
        Check if InspectorWindows are set. If they are, pause them.
        
        Pay attention not to confuse with RobotMonitorWidget.pause.
        
        :type paused: bool
        :type diagnostic_arr: DiagnosticArray
        """
        
        if paused:
            self.pause(diagnostic_arr)
        # if (not _paused and len(self._viewers) > 0):
        elif (len(self._toplv_statusitems) > 0):
            diag_array_queue = self.timeline_pane._get_diagnosticarray()
            statitems = []
            for diag_arr in diag_array_queue:
                state_instant = InstantaneousState()
                state_instant.update(diag_arr)
                statitems.append(state_instant)
        
        for statitem_toplv in self._toplv_statusitems:
            if (paused):
                statitem_toplv.disable()
            else:
                statitem_toplv.enable()    
                for state_instant in statitems:
                    all = state_instant.get_items()
                    if (all.has_key(statitem_toplv.get_name())):
                        statitem_toplv.update(
                                        all[statitem_toplv.get_name()].status)
                        
    def _update_flat_tree(self, diag_arr):
        """
        Update the given flat tree (ie. tree that doesn't show children nodes - 
        all of its elements will be shown on top level) with 
        all the DiagnosticStatus instances contained in the given DiagnosticArray, 
        regardless of the level of the device in a device category.
        
        For both warn / error trees, StatusItem instances are newly generated.
    
        :type diag_arr: DiagnosticArray
        """
        
        for diag_stat_new in diag_arr.status:
            # Num of loops here should be equal to the num of the top 
            # DiagnosticStatus item. Ex. in PR2, 9 or so.  
            
            stat_lv_new = diag_stat_new.level
            dev_name = diag_stat_new.name
            correspondent_warn_curr = Util.get_correspondent(
                                                  Util.get_nice_name(dev_name),
                                                  self._warn_statusitems)
            dev_index_warn_curr = correspondent_warn_curr[Util._DICTKEY_INDEX]
            rospy.logdebug('###_update_flat_tree dev_index_warn_curr=%s dev_name=%s',
                          dev_index_warn_curr, dev_name)         
            correspondent_err_curr = Util.get_correspondent(
                                                 Util.get_nice_name(dev_name),
                                                 self._err_statusitems)
            dev_index_err_curr = correspondent_err_curr[Util._DICTKEY_INDEX]            
            headline = "%s" % diag_stat_new.name
            if DiagnosticStatus.OK == stat_lv_new:
                if 0 <= dev_index_warn_curr:
                    rospy.logdebug('#_update_flat_tree QQQ dev_index_warn_curr=%s dev_name=%s, stat_lv_new=%d',
                          dev_index_warn_curr, dev_name, stat_lv_new)
                    statitem_curr = self._get_statitem(dev_index_warn_curr,
                                                       self._warn_statusitems,
                                                       self.warn_tree, 1)
                    statitem_curr.warning_id = None
                elif 0 <= dev_index_err_curr:
                    statitem_curr = self._get_statitem(dev_index_err_curr,
                                                       self._err_statusitems,
                                                       self.err_tree, 1)
                    statitem_curr.error_id = None
            elif DiagnosticStatus.WARN == stat_lv_new:
                statitem = None
                if 0 <= dev_index_err_curr:
                    # If the corresponding statusitem is in error tree,
                    # move it to warn tree.
                    statitem = self._get_statitem(dev_index_err_curr,
                                                  self._err_statusitems,
                                                  self.err_tree)
                    self._add_statitem(statitem, self._warn_statusitems,
                                      self.warn_tree, headline, 
                                      diag_stat_new.message, stat_lv_new)
                elif (dev_index_warn_curr < 0 and dev_index_err_curr < 0):
                    # If the corresponding statusitem isn't found, 
                    # create new obj.
                    statitem = StatusItem(diag_stat_new)
                    self._add_statitem(statitem, self._warn_statusitems,
                                      self.warn_tree, headline,
                                      diag_stat_new.message, stat_lv_new)
                    self._warn_statusitems.append(statitem)
                elif (0 < dev_index_warn_curr):
                    # If the corresponding statusitem is already in warn tree,
                    # obtain the instance.
                    statitem = self._get_statitem(dev_index_warn_curr,
                                                  self._warn_statusitems)

                if statitem: # If not None
                    # Updating statusitem will keep popup window also update.
                    dict_status = statitem.update_children(diag_stat_new, 
                                                           diag_arr)
            elif ((DiagnosticStatus.ERROR == stat_lv_new) or
                  (DiagnosticStatus.STALE == stat_lv_new)):
                statitem = None
                if 0 <= dev_index_warn_curr:
                    # If the corresponding statusitem is in warn tree,
                    # move it to err tree.
                    statitem = self._get_statitem(dev_index_warn_curr,
                                                  self._warn_statusitems,
                                                  self.warn_tree)
                    self._add_statitem(statitem, self._err_statusitems,
                                      self.err_tree, headline,
                                      diag_stat_new.message, stat_lv_new)
                elif (0 <= dev_index_err_curr):
                    # If the corresponding statusitem is already in err tree,
                    # obtain the instance.
                    statitem = self._get_statitem(dev_index_err_curr,
                                                  self._err_statusitems)
                elif (dev_index_warn_curr < 0 and dev_index_err_curr < 0):
                    # If the corresponding statusitem isn't found, 
                    # create new obj.
                    statitem = StatusItem(diag_stat_new)
                    self._add_statitem(statitem, self._err_statusitems,
                                      self.err_tree, headline,
                                      diag_stat_new.message, stat_lv_new)


                if statitem: # If not None
                    # Updating statusitem will keep popup window also update.
                    dict_status = statitem.update_children(diag_stat_new, 
                                                           diag_arr)                    
        
        self._sig_tree_nodes_updated.emit(self._TREE_WARN)
        self._sig_tree_nodes_updated.emit(self._TREE_ERR)
        
    def _add_statitem(self, statusitem, statitem_list,
                      tree, headline, statusmsg, statlevel):
        
        if 'Warning' == statusmsg or 'Error' == statusmsg:
            return 
        
        statusitem.setText(0, headline)
        statusitem.setText(1, statusmsg)
        statusitem.setIcon(0, Util._IMG_DICT[statlevel])
        statitem_list.append(statusitem)                
        tree.addTopLevelItem(statusitem)
        rospy.logdebug(' _add_statitem statitem_list length=%d',
                       len(statitem_list))
           
    def _get_statitem(self, item_index, item_list, tree = None, mode = 2):
        """
        :param mode: 1 = remove from given list, 2 = w/o removing.
        """
        statitem_existing = item_list[item_index]
        if 1 == mode:
            tree.takeTopLevelItem(tree.indexOfTopLevelItem(statitem_existing))
            item_list.pop(item_index)
        return statitem_existing

    def _on_new_message_received(self, msg):
        """    
        Called whenever a new message is received by the timeline.  
        Different from new_message in that it is called even if the timeline is 
        _paused, and only when a new message is received, not when the timeline 
        is scrubbed
        """    
        self._last_message_time = rospy.get_time()

    def _update_message_state(self):
 
        current_time = rospy.get_time()
        time_diff = current_time - self._last_message_time
        rospy.logdebug('_update_message_state time_diff= %s ' + 
                       'self._last_message_time=%s', time_diff,
                       self._last_message_time)
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

    def shutdown(self):
        """
        This needs to be called whenever this class terminates.
        This closes all the instances on all trees. 
        Also unregisters ROS' subscriber, stops timer.  
        """

        rospy.logdebug('RobotMonitorWidget in shutdown')
        # Close all StatusItem (and each associated InspectWidget)        
        # self.tree_all_devices.clear()  # Doesn't work for the purpose 
                                         # (inspector windows don't get closed)              
        for item in self._err_statusitems:
            item.close()
        for item in self._warn_statusitems:
            item.close()
        for item in self._toplv_statusitems:
            item.close()

        self._sub.unregister()

        self._timer.stop()
        del self._timer           

    def save_settings(self, plugin_settings, instance_settings):
        instance_settings.set_value('splitter', self.splitter.saveState())

    def restore_settings(self, plugin_settings, instance_settings):
        if instance_settings.contains('splitter'):
            self.splitter.restoreState(instance_settings.value('splitter'))
        else:
            self.splitter.setSizes([100, 100, 200])

    def _clear(self):
        rospy.logdebug(' RobotMonitorWidget _clear called ')
        self.err_tree.clear()
        self.warn_tree.clear()

    def get_color_for_value(self, queue_diagnostic, color_index):
        """
        Overridden from AbstractStatusWidget.
        
        :type color_index: int
        """
        
        len_q = len(queue_diagnostic)
        rospy.logdebug(' get_color_for_value color_index=%d len_q=%d',
                      color_index, len_q)
        if (color_index == 1 and len_q == 0): #TODO Needs to be reverted back.
        #if (color_index <= 2 and len_q == 0): # TODO This line is Debug only
            return QColor('grey')
        return Util._get_color_for_message(queue_diagnostic[color_index - 1])
                               # When _queue_diagnostic is empty,
                               # this yield error when color_index > 0. 
                               