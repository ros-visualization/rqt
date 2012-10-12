import roslib;roslib.load_manifest('rqt_robot_monitor')
import rospy
from diagnostic_msgs.msg import DiagnosticArray

from python_qt_binding.QtGui import QWidget, QVBoxLayout, QHBoxLayout, QTreeWidget, QTextCursor, QTreeWidgetItem, QTextEdit, QPushButton, QGraphicsScene, QGraphicsView, QPen, QBrush, QColor
from python_qt_binding.QtCore import pyqtSignal, Qt

from math import floor

def get_nice_name(status_name):
    return status_name.split('/')[-1]

def remove_parent_name(status_name):
    return ('/'.join(status_name.split('/')[2:])).strip()

def get_parent_name(status_name):
    return ('/'.join(status_name.split('/')[:-1])).strip()

class StatusItem(QTreeWidgetItem):
    def __init__(self, status):
        super(StatusItem, self).__init__()
    
        self.items = []
        self.name = status.name
        self.level = status.level
        self.inspector = None
        
        self.setText(0, '/' + get_nice_name(self.name))

    def get_children(self, msg):
        ret = []

        for k in msg.status:
            if k.name.startswith(self.name):
                if not k.name == self.name:
                    ret.append(k)

        return ret

    def update(self, status, msg):
        self.status = status

        if self.inspector:
            self.inspector.update(status)
        
        children = self.get_children(msg)

        names = [s.name for s in self.items]
        new_items = []
        remove = []
        for i in children:
            name = i.name
            if name in names:
                w = self.items[names.index(name)]
                w.update(i, msg)
            elif len(self.strip_child(name).split('/')) <= 2:
                sti = StatusItem(i)
                sti.update(i, msg)
                self.items.append(sti)
                new_items.append(sti)
        self.addChildren(new_items)

    def on_click(self):
        if not self.inspector:
            self.inspector = InspectorWidget(self.status)
        else:
            self.inspector.activateWindow()

    def strip_child(self, child):
        return child.replace(self.name, '')

class Snapshot(QTextEdit):
    """Display a single static status message. Helps facilitate copy/paste"""
    def __init__(self, status):
        super(Snapshot, self).__init__()

        self.write("Full Name", status.name)
        self.write("Component", status.name.split('/')[-1])
        self.write("Hardware ID", status.hardware_id)
        self.write("Level", status.level)
        self.write("Message", status.message)
        self.insertPlainText('\n')

        for value in status.values:
            self.write(value.key, value.value)

        self.setGeometry(0,0, 300, 400)
        self.show()

    def write(self, k, v):
        self.setFontWeight(75)
        self.insertPlainText(str(k))
        self.insertPlainText(': ')
     
        self.setFontWeight(50)
        self.insertPlainText(str(v))
        self.insertPlainText('\n')           

class InspectorWidget(QWidget):
    write = pyqtSignal(str, str)
    newline = pyqtSignal()
    clear = pyqtSignal()
    def __init__(self, status):
        super(InspectorWidget, self).__init__()
        self.status = status
        self.setWindowTitle(status.name)
        self.paused = False

        layout = QVBoxLayout()
        
        self.disp = QTextEdit()
        self.snapshot = QPushButton("Snapshot")

        self.time = TimelineWidget(self)

        layout.addWidget(self.disp, 1)
        layout.addWidget(self.time, 0)
        layout.addWidget(self.snapshot)

        self.snaps = []
        self.snapshot.clicked.connect(self.take_snapshot)

        self.write.connect(self.write_kv)
        self.newline.connect(lambda: self.disp.insertPlainText('\n'))
        self.clear.connect(lambda: self.disp.clear())

        self.setLayout(layout)
        self.setGeometry(0,0,300,400)
        self.show()
        self.update(status)

    def write_kv(self, k, v):
        self.disp.setFontWeight(75)
        self.disp.insertPlainText(k)
        self.disp.insertPlainText(': ')

        self.disp.setFontWeight(50)
        self.disp.insertPlainText(v)
        self.disp.insertPlainText('\n')

    def pause(self, msg):
        self.update(msg);
        self.paused = True

    def unpause(self):
        self.paused = False

    def update(self, status):
        if not self.paused:
            self.status = status
            self.time.add_message(status)

            self.clear.emit()
            self.write.emit("Full Name", status.name)
            self.write.emit("Component", status.name.split('/')[-1])
            self.write.emit("Hardware ID", status.hardware_id)
            self.write.emit("Level", str(status.level))
            self.write.emit("Message", status.message)
            self.newline.emit()

            for v in status.values:
                self.write.emit(v.key, v.value)

    def take_snapshot(self):
        snap = Snapshot(self.status)
        self.snaps.append(snap)

class TimelineWidget(QWidget):
    class TimelineView(QGraphicsView):
        def __init__(self, parent):
            super(TimelineWidget.TimelineView, self).__init__()
            self.parent = parent

        def mouseReleaseEvent(self, event):
            self.parent.mouse_release(event)

    update = pyqtSignal()
    def __init__(self, parent):
        super(TimelineWidget, self).__init__()
        self.parent = parent

        self._layout = QHBoxLayout()

        #self._view = QGraphicsView()
        self._view = TimelineWidget.TimelineView(self)

        self._scene = QGraphicsScene()
        self._colors = [QColor('green'), QColor('yellow'), QColor('red')]

        self._messages = [None for x in range(20)]
        self._mq = [1 for x in range(20)] 

        self._view.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self._view.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self._view.setScene(self._scene)
        self._layout.addWidget(self._view, 1)

        self.pause_button = QPushButton('Pause')
        self.pause_button.setCheckable(True)
        self.pause_button.clicked.connect(self.pause)
        self._layout.addWidget(self.pause_button)

        self.setLayout(self._layout)

        self.update.connect(self.redraw)

    def redraw(self):
        self._scene.clear()
        self._scene
        for i, m in enumerate(self._mq):
            w = float(self._view.viewport().width())/len(self._mq)
            h = self._view.viewport().height()
            rect = self._scene.addRect(w*i, 0, w, h, QColor('black'), self._colors[m])

    def mouse_release(self, event):
        i = int(floor(event.x()/(float(self._view.viewport().width())/len(self._mq))))

        msg = self._messages[i]
        if msg:
            self.parent.pause(msg)
            if not self.pause_button.isChecked():
                self.pause_button.toggle()

    def resizeEvent(self, event):
        self.redraw()

    def get_worst(self, msg):
        lvl = 0
        for status in msg.status:
            if status.level > lvl:
                lvl = status.level

        return lvl

    def add_message(self, msg):
        self._messages = self._messages[1:]
        self._messages.append(msg)

        self._mq = self._mq[1:]
        try:
            lvl = msg.level
        except AttributeError:
            lvl = self.get_worst(msg)

        if lvl > 2:
            lvl = 2

        self._mq.append(lvl)

        self.update.emit()

    def pause(self, state):
        if state:
            self.parent.pause(self._messages[-1])
        else:
            self.parent.unpause()

class RobotMonitor(QWidget):
    sig_err = pyqtSignal(str)
    sig_warn = pyqtSignal(str)
    sig_clear = pyqtSignal()

    def __init__(self, topic):
        super(RobotMonitor, self).__init__()
        self.setObjectName('Robot Monitor')

        self.top_items = []
        layout = QVBoxLayout()

        self.err = QTreeWidget()
        self.err.setHeaderLabel("Errors")
        self.warn = QTreeWidget()
        self.warn.setHeaderLabel("Warnings")

        self.sig_clear.connect(self.clear)
        self.sig_err.connect(self.disp_err)
        self.sig_warn.connect(self.disp_warn)

        self.comp = QTreeWidget()
        self.comp.setHeaderLabel("All")
        self.comp.itemDoubleClicked.connect(self.tree_clicked)

        self.time = TimelineWidget(self)

        layout.addWidget(self.err)
        layout.addWidget(self.warn)
        layout.addWidget(self.comp, 1)
        layout.addWidget(self.time, 0)

        self.setLayout(layout)

        self.paused = False

        self.topic = topic
        self.sub = rospy.Subscriber(self.topic, DiagnosticArray, self.cb)
        self.setWindowTitle('Robot Monitor')

    def cb(self, msg):
        if not self.paused:
            self.sig_clear.emit()
            self.update_tree(msg)
            self.update_we(msg)
            self.time.add_message(msg)

    def tree_clicked(self, item, yes):
        item.on_click()

    def update_tree(self, msg):
        #Update the tree from the bottom

        names = [get_nice_name(k.name) for k in self.top_items]
        add = []
        for i in self._top_level(msg):
            name = get_nice_name(i.name)
            if name in names:
                self.top_items[names.index(name)].update(i, msg)
            else:
                nw = StatusItem(i)
                nw.update(i, msg)
                self.top_items.append(nw)
                add.append(nw)
        
        self.comp.addTopLevelItems(add)
        
    def _top_level(self, msg):
        ret = []
        for i in msg.status:
            if len(i.name.split('/')) == 2:
                ret.append(i)
        
        return ret

    def update_we(self, msg):
        """Update the warning and error boxes"""
        for status in msg.status:
            if status.level == status.WARN:
                txt = "%s : %s"%(status.name, status.message)
                self.sig_warn.emit(txt)
            elif status.level == status.ERROR:
                txt = "%s : %s"%(status.name, status.message)
                self.sig_err.emit(txt)

    def pause(self, msg):
        self.paused = True
        self.sig_clear.emit()
        self.update_we(msg)
        self.update_tree(msg)

    def unpause(self):
        self.paused = False

    def clear(self):
        self.err.clear()
        self.warn.clear()

    def disp_err(self, msg):
        i = QTreeWidgetItem()
        i.setText(0, msg)
        self.err.addTopLevelItem(i)
        
    def disp_warn(self,msg):
        i = QTreeWidgetItem()
        i.setText(0, msg)
        self.warn.addTopLevelItem(i)

    def close(self):
        if self.sub:
            self.sub.unregister()
            self.sub = None

