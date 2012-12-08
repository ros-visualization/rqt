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

import os

from python_qt_binding.QtCore import Signal
from python_qt_binding.QtGui import QToolButton
import rospy

from .util import IconHelper


class IconToolButton(QToolButton):
    """
    This is the base class for all widgets.
    It provides state and icon switching support as well as convenience functions for creating icons.

    :raises IndexError: if ``icons`` is not a list of lists of strings

    :param name: name of the object
    :type name: str
    :param icons: A list of lists of strings to create icons for the states of this button.
    If only one is supplied then ok, warn, error, stale icons will be created with overlays
    :type icons: list
    :param clicked_icons: A list of clicked state icons. len must equal icons
    :type clicked_icons: list
    :param suppress_overlays: if false and there is only one icon path supplied
    :type suppress_overlays: bool
    :param icon_paths: list of lists of package and subdirectory in the form
    ['package name', 'subdirectory'] example ['rqt_pr2_dashboard', 'images/svg']
    :type icon_paths: list of lists of strings
    """
    state_changed = Signal(int)

    def __init__(self, name, icons, clicked_icons=None, suppress_overlays=False, icon_paths=None):
        super(IconToolButton, self).__init__()

        self.name = name
        self.setObjectName(self.name)

        self.state_changed.connect(self._update_state)
        self.pressed.connect(self._pressed)
        self.released.connect(self._released)

        import rospkg
        icon_paths = (icon_paths if icon_paths else []) + [['rqt_robot_dashboard', 'images']]
        paths = []
        for path in icon_paths:
            paths.append(os.path.join(rospkg.RosPack().get_path(path[0]), path[1]))
        self.icon_helper = IconHelper(paths)

        self.setStyleSheet('QToolButton {border: none;}')

        self.__state = 0
        self.set_icon_lists(icons, clicked_icons, suppress_overlays)

    def update_state(self, state):
        """
        Set the state of this button.
        This will also update the icon for the button based on the ``self._icons`` list

        :raises IndexError: If state is not a proper index to ``self._icons``

        :param state: The state to set.
        :type state: int
        """
        if 0 <= state and state < len(self._icons):
            self.__state = state
            self.state_changed.emit(self.__state)
        else:
            raise IndexError("%s update_state received invalid state: %s" % (self.name, state))

    def set_icon_lists(self, icons, clicked_icons=None, suppress_overlays=False):
        """
        Sets up the icon lists for the button states.
        There must be one index in icons for each state.

        :raises IndexError: if ``icons`` is not a list of lists of strings

        :param icons: A list of lists of strings to create icons for the states of this button.
        If only one is supplied then ok, warn, error, stale icons will be created with overlays
        :type icons: list
        :param clicked_icons: A list of clicked state icons. len must equal icons
        :type clicked_icons: list
        :param suppress_overlays: if false and there is only one icon path supplied
        :type suppress_overlays: bool

        """
        if clicked_icons is not None and len(icons) != len(clicked_icons):
            rospy.logerr("%s: icons and clicked states are unequal" % self.name)
            icons = clicked_icons = ['ic-missing-icon.svg']
        if not (type(icons) is list and type(icons[0]) is list and type(icons[0][0] is str)):
            raise(IndexError("icons must be a list of lists of strings"))
        if len(icons) <= 0:
            rospy.logerr("%s: Icons not supplied" % self.name)
            icons = clicked_icons = ['ic-missing-icon.svg']
        if len(icons) == 1 and suppress_overlays == False:
            if icons[0][0][-4].lower() == '.svg':
                icons.append(icons[0] + ['ol-warn-badge.svg'])
                icons.append(icons[0] + ['ol-err-badge.svg'])
                icons.append(icons[0] + ['ol-stale-badge.svg'])
            else:
                icons.append(icons[0] + ['warn-overlay.png'])
                icons.append(icons[0] + ['err-overlay.png'])
                icons.append(icons[0] + ['stale-overlay.png'])
        if clicked_icons is None:
            clicked_icons = []
            for name in icons:
                clicked_icons.append(name + ['ol-click.svg'])
        self._icons = []
        for icon in icons:
            self._icons.append(self.icon_helper.build_icon(icon))
        self._clicked_icons = []
        for icon in clicked_icons:
            self._clicked_icons.append(self.icon_helper.build_icon(icon))

    def _update_state(self, state):
        if self.isDown():
            self.setIcon(self._clicked_icons[self.__state])
        else:
            self.setIcon(self._icons[self.__state])

    def _pressed(self):
        self.setIcon(self._clicked_icons[self.__state])

    def _released(self):
        self.setIcon(self._icons[self.__state])
