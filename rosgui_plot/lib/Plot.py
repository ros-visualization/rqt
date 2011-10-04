#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os

from rosgui.QtBindingHelper import import_from_qt, loadUi
Qt, QTimer, Signal, Slot, qDebug = import_from_qt(['Qt', 'QTimer', 'Signal', 'Slot', 'qDebug'], 'QtCore')
QDockWidget = import_from_qt('QDockWidget', 'QtGui')

import roslib
roslib.load_manifest('rosgui_plot')
import rospy
from std_msgs.msg import Float32, Float64, Int16, Int32, Int64
from rxtools.rosplot import ROSData 
from DataPlot import DataPlot

# main class inherits from the ui window class
class Plot(QDockWidget):

    def __init__(self, parent, plugin_context):
        QDockWidget.__init__(self, plugin_context.main_window())
        self.setObjectName('Plot')

        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'Plot.ui')
        loadUi(ui_file, self, {'DataPlot': DataPlot})
        self.start_time = rospy.get_time()
        self.rosdata_ = {}
        self.on_buttonRefreshTopicList_clicked()

        # setup drag 'n drop
        self.data_plot.dropEvent = self.dropEvent
        self.data_plot.dragEnterEvent = self.dragEnterEvent

        # setup data plot 
        self.data_plot.redrawManually = True
        self.pause_button.clicked[bool].connect(self.data_plot.togglePause)

        # add our self to the main window
        plugin_context.main_window().addDockWidget(Qt.RightDockWidgetArea, self)

        # init and start update timer
        self.timerUpdatePlot = QTimer(self)
        self.timerUpdatePlot.timeout.connect(self.updatePlot)
        self.timerUpdatePlot.start(40)

    
    def updatePlot(self):
        for topic_name, rosdata in self.rosdata_.items():
            # TODO: use data_x as time stamp
            data_x, data_y = rosdata.next()
            for value in data_y:
                self.data_plot.updateValue(topic_name, value)
        self.data_plot.redraw()


    @Slot('QDragEnterEvent*')
    def dragEnterEvent(self, event):
        if not event.mimeData().hasText():
            if not hasattr(event.source(), 'selectedItems') or len(event.source().selectedItems()) == 0:
                qDebug('Plot.dragEnterEvent(): not hasattr(event.source(), selectedItems) or len(event.source().selectedItems()) == 0')
                return
            item = event.source().selectedItems()[0]
            ros_topic_name = item.data(0, Qt.UserRole)
            if ros_topic_name == None:
                qDebug('Plot.dragEnterEvent(): not hasattr(item, ros_topic_name_)')
                return
        # TODO: do some checks for valid numeric topic here
        event.acceptProposedAction()


    @Slot('QDropEvent*')
    def dropEvent(self, event):
        if event.mimeData().hasText():
            topic_name = str(event.mimeData().text())
        else:
            droped_item = event.source().selectedItems()[0]
            topic_name = str(droped_item.data(0, Qt.UserRole))
        self.addTopic(topic_name)


    @Slot()
    def on_buttonRefreshTopicList_clicked(self):
        self.topic_combo_box.clear()
        topic_list = rospy.get_published_topics()
        for topic_name, message_type in topic_list:
            message_class = roslib.message.get_message_class(message_type)
            if (message_class in [Float32, Float64, Int16, Int32, Int64]):
                self.topic_combo_box.addItem(topic_name + '/data')
        

    @Slot()
    def on_subscribe_topic_button_clicked(self):
        self.addTopic(str(self.topic_combo_box.currentText()))
        
        
    def addTopic(self, topic_name):
        if topic_name in self.rosdata_:
            qDebug('Plot.on_subscribe_topic_button_clicked(): topic already subscribed: %s' % topic_name)
            return
        
        self.data_plot.addCurve(topic_name, topic_name)
        self.rosdata_[topic_name] = ROSData(topic_name, self.start_time)


    @Slot()
    def on_buttonClear_clicked(self):
        self.clean_up_subscribers()
        
        
    def clean_up_subscribers(self):
        for topic_name, rosdata in self.rosdata_.items():
            rosdata.close()
            self.data_plot.removeCurve(topic_name)
        self.rosdata_ = {}
            

    def save_settings(self, global_settings, perspective_settings):
        #perspective_settings.set_value('publishing_', self.publishing_)
        pass


    def restore_settings(self, global_settings, perspective_settings):
        #self.publishing_ = perspective_settings.value('publishing_') in [True, 'true']
        pass


    def set_name(self, name):
        self.setWindowTitle(name) 


    # override Qt's closeEvent() method to trigger plugin unloading
    def closeEvent(self, event):
        QDockWidget.closeEvent(self, event)
        if event.isAccepted():
            self.deleteLater()


    def close_plugin(self):
        self.clean_up_subscribers()
        QDockWidget.close(self)
