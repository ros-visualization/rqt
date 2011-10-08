#!/usr/bin/env python
import sys

def pyside(data):
    # register PySide modules
    import PySide.QtCore, PySide.QtGui, PySide.QtOpenGL
    sys.modules['QtCore'] = PySide.QtCore
    sys.modules['QtGui'] = PySide.QtGui
    sys.modules['QtOpenGL'] = PySide.QtOpenGL

    # set some names for compatibility with PyQt4
    sys.modules['QtCore'].pyqtSignal = sys.modules['QtCore'].Signal
    sys.modules['QtCore'].pyqtSlot = sys.modules['QtCore'].Slot
    sys.modules['QtCore'].pyqtProperty = sys.modules['QtCore'].Property

    data['qt_package'] = 'PySide'
    data['version'] = PySide.__version__

    # try to register PySideQwt module
    try:
        import PySideQwt
        sys.modules['Qwt'] = PySideQwt
    except ImportError:
        pass


def pyqt(data):
    # select PyQt4 API
    import sip
    sip.setapi('QString', 2)
    sip.setapi('QVariant', 2)

    # register PyQt4 modules
    import PyQt4.QtCore, PyQt4.QtGui, PyQt4.QtOpenGL
    sys.modules['QtCore'] = PyQt4.QtCore
    sys.modules['QtGui'] = PyQt4.QtGui
    sys.modules['QtOpenGL'] = PyQt4.QtOpenGL

    # set some names for compatibility with PySide
    sys.modules['QtCore'].Signal = sys.modules['QtCore'].pyqtSignal
    sys.modules['QtCore'].Slot = sys.modules['QtCore'].pyqtSlot
    sys.modules['QtCore'].Property = sys.modules['QtCore'].pyqtProperty

    data['version'] = PyQt4.QtCore.PYQT_VERSION_STR
    data['qt_package'] = 'PyQt4'

    # try to register PyQt4.Qwt5 module
    try:
        import PyQt4.Qwt5
        sys.modules['Qwt'] = PyQt4.Qwt5
    except ImportError:
        pass


# order of bindings can be changed here
_ORDER_OF_BINDINGS = [pyqt, pyside]

_selected_qt_binding = {
    'id': None,
    'qt_package': None,
    'version': None,
}

# try to load any Qt binding
for binding in _ORDER_OF_BINDINGS:
    try:
        binding(_selected_qt_binding)
        _selected_qt_binding['id'] = binding.__name__
        break
    except ImportError:
        pass

if _selected_qt_binding['id'] is None:
    bindings = [binding.__name__ for binding in _ORDER_OF_BINDINGS]
    raise ImportError('Could not find any Qt binding (looked for %s)' % bindings)

QT_BINDING = _selected_qt_binding['id']
print 'QtBindingHelper: using %s' % QT_BINDING


if _selected_qt_binding['id'] == 'pyside':

    def loadUi(uifile, baseinstance=None, user_classes={}):
        from PySide.QtUiTools import QUiLoader
        from PySide.QtCore import QMetaObject

        class CustomUiLoader(QUiLoader):
            def __init__(self, baseinstance):
                super(CustomUiLoader, self).__init__()
                self.baseinstance = baseinstance

            def createWidget(self, className, parent=None, name=""):
                if user_classes.has_key(className):
                    widget = user_classes[className](parent)
                else:
                    widget = QUiLoader.createWidget(self, className, parent, name)
                if str(type(widget)).find(className) < 0:
                    sys.modules['QtCore'].qDebug(str('PySide.loadUi(): could not find widget class "%s", defaulting to "%s"' % (className, type(widget))))
                if parent is None:
                    return self.baseinstance
                else:
                    setattr(self.baseinstance, name, widget)
                    return widget

        loader = CustomUiLoader(baseinstance)
        ui = loader.load(uifile)
        QMetaObject.connectSlotsByName(ui)
        return ui


elif _selected_qt_binding['id'] == 'pyqt':

    def loadUi(uifile, baseinstance=None, user_classes={}):
        from PyQt4 import uic
        return uic.loadUi(uifile, baseinstance=baseinstance)


if __name__ == "__main__":
    pass

