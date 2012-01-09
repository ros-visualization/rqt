# Copyright (c) 2011, Dirk Thomas, Dorian Scholz, TU Darmstadt
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the TU Darmstadt nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import QtBindingHelper #@UnusedImport
from QtCore import QEvent, QPoint, Qt
from QtGui import QApplication, QDockWidget, QMainWindow, QMouseEvent

from ReparentEvent import ReparentEvent

class DockWidget(QDockWidget):

    """Dock widget with the capability to be reparented via drag-and-drop to any other main window."""

    def __init__(self, container_manager):
        super(DockWidget, self).__init__()
        self._container_manager = container_manager
        if self._container_manager is not None:
            self.event = self._event
        self._dragging = False

    def _event(self, e):
        if not self._dragging and e.type() == QEvent.Move and QApplication.mouseButtons() & Qt.LeftButton:
            if QApplication.widgetAt(e.pos()) is not None:
                #print 'DockWidget.event()', 'start drag', self.windowTitle(), e.pos()
                self._dragging = True
                # ignore further mouse events so that the widget behind this dock widget can be determined easily
                self.setAttribute(Qt.WA_TransparentForMouseEvents)

        if self._dragging and e.type() == QEvent.MouseButtonRelease and e.button() == Qt.LeftButton:
            #print 'DockWidget.event()', 'stop drag', self.windowTitle(), e.globalPos()
            self._dragging = False
            self.setAttribute(Qt.WA_TransparentForMouseEvents, False)

        if self._dragging and e.type() == QEvent.MouseMove and e.buttons() & Qt.LeftButton:
            widget = QApplication.widgetAt(e.globalPos())
            while widget is not None:
                #print widget.objectName()
                if isinstance(widget, QMainWindow):
                    break
                widget = widget.parent()
            child = self.main_window if hasattr(self, 'main_window') else None
            if widget is not None and widget != self.parent() and (child is None or widget != child):
                # while moving over an other main window
                #print 'DockWidget._event()', 'MouseMove', 'reparent while dragging'

                # schedule stop of pseudo drag'n'drop
                mouse_release_event = QMouseEvent(QEvent.MouseButtonRelease, e.pos(), Qt.LeftButton, Qt.NoButton, e.modifiers())
                QApplication.instance().postEvent(self, mouse_release_event)
                # schedule reparent to hovered main window
                reparent_event = ReparentEvent(self, widget)
                QApplication.instance().postEvent(self._container_manager, reparent_event)

                # let the reparent complete
                QApplication.sendPostedEvents()

                # schedule restart of pseudo drag'n'drop
                mouse_repress_event = QMouseEvent(QEvent.MouseButtonPress, QPoint(1, 1), Qt.LeftButton, Qt.LeftButton, e.modifiers())
                QApplication.instance().postEvent(self, mouse_repress_event)
                # let the repress complete
                QApplication.sendPostedEvents()

                # schedule move to trigger pseudo drag'n'drop
                mouse_move_event = QMouseEvent(QEvent.MouseMove, QPoint(2 + QApplication.startDragDistance(), 1), Qt.NoButton, Qt.LeftButton, e.modifiers())
                QApplication.instance().postEvent(self, mouse_move_event)

        return super(DockWidget, self).event(e)

    def save_settings(self, perspective_settings):
        perspective_settings.set_value('parent', self._parent_container_serial_number())

        title_bar = self.titleBarWidget()
        title_bar.save_settings(perspective_settings)

    def restore_settings(self, perspective_settings):
        serial_number = perspective_settings.value('parent', None)
        if serial_number is not None:
            serial_number = int(serial_number)
        if self._parent_container_serial_number() != serial_number and self._container_manager is not None:
            new_parent = self._container_manager.get_container(serial_number)
            area = self.parent().dockWidgetArea(self)
            if new_parent is not None:
                new_parent.main_window.addDockWidget(area, self)
            else:
                self._main_window.addDockWidget(area, self)

        title_bar = self.titleBarWidget()
        title_bar.restore_settings(perspective_settings)

    def _parent_container_serial_number(self):
        from DockWidgetContainer import DockWidgetContainer
        serial_number = None
        parent = self.parent()
        if parent is not None:
            parent = parent.parent()
            if isinstance(parent, DockWidgetContainer):
                serial_number = parent.serial_number()
        return serial_number
