#!/usr/bin/env python

def pyside(data):
    import PySide.QtCore as _QtCore
    data['qt_package'] = 'PySide'
    data['QtCore'] = _QtCore
    try:
        import PySideQwt as _Qwt
        data['Qwt'] = _Qwt
    except ImportError:
        pass

def pyqt(data):
    import sip
    sip.setapi('QString', 2)
    sip.setapi('QVariant', 2)
    import PyQt4.QtCore as _QtCore
    data['qt_package'] = 'PyQt4'
    data['QtCore'] = _QtCore
    try:
        import PyQt4.Qwt5 as _Qwt
        data['Qwt'] = _Qwt
    except ImportError:
        pass

# order of bindings can be changed here
_ORDER_OF_BINDINGS = [pyqt, pyside]

_selected_qt_binding = {
    'id': None,
    'qt_package': None,
    'QtCore': None,
    'Qwt': None,
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
    bindings = []
    for binding in _ORDER_OF_BINDINGS:
        bindings.append(binding.__name__)
    raise ImportError('Could not find any Qt binding (looked for %s)' % bindings)

print 'QtBindingHelper: using %s' % _selected_qt_binding['id']


QT_BINDING = _selected_qt_binding['id']
QtCore = _selected_qt_binding['QtCore']
if _selected_qt_binding['Qwt'] is not None:
    Qwt = _selected_qt_binding['Qwt']


def import_from_qt(attributes, module_name = None):
    package_hirachy = [_selected_qt_binding['qt_package']]
    if module_name:
        package_hirachy.append(module_name)
    package_name = '.'.join(package_hirachy)

    if isinstance(attributes, (list, tuple)) and len(attributes) == 1:
        attributes = attributes[0]

    #print 'QtBindingHelper: from %s import %s' % (package_name, attributes)
    if isinstance(attributes, (list, tuple)):
        module = __import__(package_name, globals(), locals(), attributes, -1)
        return [getattr(module, attribute) for attribute in attributes]
    else:
        module = __import__(package_name, globals(), locals(), [attributes], -1)
        return getattr(module, attributes)


if _selected_qt_binding['id'] == 'pyside':
    def loadUi(uifile, baseinstance = None, user_classes = {}):
        from PySide.QtUiTools import QUiLoader
        from PySide.QtCore import QMetaObject

        class CustomUiLoader(QUiLoader):
            def __init__(self, baseinstance):
                super(CustomUiLoader, self).__init__()
                self.baseinstance = baseinstance

            def createWidget(self, className, parent = None, name = ""):
                if user_classes.has_key(className):
                    widget = user_classes[className](parent)
                else:
                    widget = QUiLoader.createWidget(self, className, parent, name)
                if str(type(widget)).find(className) < 0:
                    QtCore.qDebug(str('PySide.loadUi(): could not find widget class "%s", defaulting to "%s"' % (className, type(widget))))
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
    QtCore.Signal = QtCore.pyqtSignal
    QtCore.Slot = QtCore.pyqtSlot
    QtCore.Property = QtCore.pyqtProperty

    def loadUi(uifile, baseinstance = None, user_classes = {}):
        from PyQt4 import uic
        return uic.loadUi(uifile, baseinstance = baseinstance)


if __name__ == "__main__":
    pass

