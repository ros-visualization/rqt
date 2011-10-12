#!/usr/bin/env python

import sys

def select_qt_binding(binding_name=None):
    global QT_BINDING, QT_BINDING_VERSION

    # order of default bindings can be changed here
    DEFAULT_BINDING_ORDER = [pyqt, pyside]

    # determine binding preference
    if binding_name is not None:
        bindings = dict((binding.__name__, binding) for binding in DEFAULT_BINDING_ORDER)
        if binding_name not in bindings:
            raise ImportError('Qt binding "%s" is unknown' % binding_name)
        DEFAULT_BINDING_ORDER = [bindings[binding_name]]

    # try to load preferred bindings
    for binding in DEFAULT_BINDING_ORDER:
        try:
            QT_BINDING_VERSION = binding()
            QT_BINDING = binding.__name__
            break
        except ImportError:
            pass

    if QT_BINDING is None:
        bindings = [binding.__name__ for binding in DEFAULT_BINDING_ORDER]
        raise ImportError('Could not find Qt binding (looked for "%s")' % bindings)

    print 'QtBindingHelper: using %s' % QT_BINDING


def pyqt():
    # select PyQt4 API
    import sip
    sip.setapi('QString', 2)
    sip.setapi('QVariant', 2)
    sip.setapi('QDate', 2)
    sip.setapi('QDateTime', 2)
    sip.setapi('QTextStream', 2)
    sip.setapi('QUrl', 2)
    sip.setapi('QTime', 2)

    # register PyQt4 modules
    import PyQt4.QtCore, PyQt4.QtGui, PyQt4.QtOpenGL
    sys.modules['QtCore'] = PyQt4.QtCore
    sys.modules['QtGui'] = PyQt4.QtGui
    sys.modules['QtOpenGL'] = PyQt4.QtOpenGL

    # set some names for compatibility with PySide
    sys.modules['QtCore'].Signal = sys.modules['QtCore'].pyqtSignal
    sys.modules['QtCore'].Slot = sys.modules['QtCore'].pyqtSlot
    sys.modules['QtCore'].Property = sys.modules['QtCore'].pyqtProperty

    # try to register PyQt4.Qwt5 module
    try:
        import PyQt4.Qwt5
        sys.modules['Qwt'] = PyQt4.Qwt5
    except ImportError:
        pass

    global loadUi
    def loadUi(uifile, baseinstance=None, user_classes={}):
        from PyQt4 import uic
        return uic.loadUi(uifile, baseinstance=baseinstance)

    return PyQt4.QtCore.PYQT_VERSION_STR


def pyside():
    # register PySide modules
    import PySide.QtCore, PySide.QtGui, PySide.QtOpenGL
    sys.modules['QtCore'] = PySide.QtCore
    sys.modules['QtGui'] = PySide.QtGui
    sys.modules['QtOpenGL'] = PySide.QtOpenGL

    # set some names for compatibility with PyQt4
    sys.modules['QtCore'].pyqtSignal = sys.modules['QtCore'].Signal
    sys.modules['QtCore'].pyqtSlot = sys.modules['QtCore'].Slot
    sys.modules['QtCore'].pyqtProperty = sys.modules['QtCore'].Property

    # try to register PySideQwt module
    try:
        import PySideQwt
        sys.modules['Qwt'] = PySideQwt
    except ImportError:
        pass

    global loadUi
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

    return PySide.__version__


QT_BINDING = None
QT_BINDING_VERSION = None

select_qt_binding(getattr(sys, 'SELECT_QT_BINDING', None))

