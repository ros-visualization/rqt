import os
from rosgui.QtBindingHelper import import_from_qt, loadUi
Slot, Qt, QEvent, QObject, qDebug = import_from_qt(['Slot', 'Qt', 'QEvent', 'QObject', 'qDebug'], 'QtCore')
QWidget, QDockWidget = import_from_qt(['QWidget', 'QDockWidget'], 'QtGui')


class RosguiDockWidgetTitleBar(QWidget):
    
    def __init__(self, dockwidget):
        super(RosguiDockWidgetTitleBar, self).__init__(dockwidget)
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'RosguiDockWidgetTitleBar.ui')
        loadUi(ui_file, self)
        self.extraButtons = {
            'help': self.buttonHelp,
            'reload': self.buttonReload,
            'settings': self.buttonSettings,
        }
        
        self.buttonFloat.clicked.connect(self.toggleFloating)
        self.buttonDockable.clicked.connect(self.toggleDockable)
        self.buttonClose.clicked.connect(dockwidget.close)
  
        dockwidget.featuresChanged.connect(self.featuresChanged)
        self.featuresChanged(0)

        self.updateTitle()
        
        self.eventCallbacks = {
            QEvent.WindowTitleChange: self.updateTitle,
        }
        dockwidget.installEventFilter(self)


    def connectButton(self, button_id, callback):
        button = self.extraButtons.get(button_id, None)
        if button is None:
            qDebug('RosguiDockWidgetTitleBar.connectButton(): unknown button_id: %s' % button_id)
            return
        button.clicked.connect(callback)


    def showButton(self, button_id, visibility = True):
        button = self.extraButtons.get(button_id, None)
        if button is None:
            qDebug('RosguiDockWidgetTitleBar.showButton(): unknown button_id: %s' % button_id)
            return
        button.setVisible(visibility)

        
    def hideButton(self, button_id):
        self.showButton(button_id, False)
        

    def eventFilter(self, obj, event):
        if self.eventCallbacks.has_key(event.type()):
            ret_val = self.eventCallbacks[event.type()](obj, event)
            if ret_val is not None:
                return ret_val
        return QObject.eventFilter(self, obj, event)

    
    def updateTitle(self, *args):
        self.labelTitle.setText(self.parentWidget().windowTitle())

    
    @Slot(bool)
    def toggleDockable(self, enabled):
        dock_widget = self.parentWidget()
        if enabled:
            dock_widget.setAllowedAreas(Qt.AllDockWidgetAreas)
        else:
            dock_widget.setAllowedAreas(Qt.NoDockWidgetArea)
            dock_widget.setFloating(True)


    def toggleFloating(self):
        dock_widget = self.parentWidget()
        floating = not dock_widget.isFloating()
        dock_widget.setFloating(floating)
        # if widget was docked, make sure the dockable button's state reflects this
        if not floating:
            self.buttonDockable.setChecked(True)
            self.toggleDockable(True)

  
    def featuresChanged(self, _features):
        features = self.parentWidget().features()
        self.buttonClose.setVisible(bool(features & QDockWidget.DockWidgetClosable))
        self.buttonFloat.setVisible(bool(features & QDockWidget.DockWidgetFloatable))


if __name__ == '__main__':
    import sys
    Qt = import_from_qt(['Qt'], 'QtCore')
    QApplication, QMainWindow, QDockWidget = import_from_qt(['QApplication', 'QMainWindow', 'QDockWidget'], 'QtGui')
    
    app = QApplication(sys.argv)
    
    win = QMainWindow()
    
    dock1 = QDockWidget('dockwidget1', win)
    win.addDockWidget(Qt.LeftDockWidgetArea, dock1)
    title_bar = RosguiDockWidgetTitleBar(dock1)
    dock1.setTitleBarWidget(title_bar)
    
    dock2 = QDockWidget('dockwidget2')
    win.addDockWidget(Qt.RightDockWidgetArea, dock2)
    title_bar = RosguiDockWidgetTitleBar(dock2)
    dock2.setTitleBarWidget(title_bar)
    
    win.resize(640, 480)
    win.show()

    app.exec_()
