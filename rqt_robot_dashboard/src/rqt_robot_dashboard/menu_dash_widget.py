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

from QtGui import QMenu, QToolButton
from .icon_tool_button import IconToolButton


class MenuDashWidget(IconToolButton):
    """
    A widget which displays a pop-up menu when clicked

    :param name: The name to give this widget.
    :type name: str
    :param icon: The icon to display in this widgets button.
    :type icon: str
    """
    def __init__(self, name, icons=None, clicked_icons=None, icon_paths=[]):
        if icons == None:
            icons = [['bg-grey.svg', 'ic-motors.svg']]
        super(MenuDashWidget, self).__init__(name, icons=icons, suppress_overlays=True, icon_paths=icon_paths)
        self.setStyleSheet('QToolButton::menu-indicator {image: url(none.jpg);} QToolButton {border: none;}')
        self.setPopupMode(QToolButton.InstantPopup)
        self.update_state(0)

        self.pressed.disconnect(self._pressed)
        self.released.disconnect(self._released)

        self._menu = QMenu()
        self._menu.aboutToHide.connect(self._released)
        self._menu.aboutToShow.connect(self._pressed)

        self.setMenu(self._menu)

    def add_separator(self):
        return self._menu.addSeparator()

    def add_action(self, name, callback):
        """
        Add an action to the menu, and return the newly created action.

        :param name: The name of the action.
        :type name: str
        :param callback: Function to be called when this item is pressed.
        :type callback: callable
        """
        return self._menu.addAction(name, callback)
