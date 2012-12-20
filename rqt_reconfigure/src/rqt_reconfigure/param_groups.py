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

from python_qt_binding.QtCore import Qt
from python_qt_binding.QtGui import QFormLayout, QGroupBox, QLabel, QPushButton, QTabWidget, QWidget
import rospy

from .param_editors import BooleanEditor, DoubleEditor, EditorWidget, EDITOR_TYPES, EnumEditor, IntegerEditor, StringEditor

_GROUP_TYPES = {
    '': 'BoxGroup',
    'collapse': 'CollapseGroup',
    'tab': 'TabGroup',
    'hide': 'HideGroup',
    'apply': 'ApplyGroup',
}

def find_cfg(config, name):
    """
    (Ze'ev) reaaaaallly cryptic function which returns the config object for 
    specified group.
    """
    cfg = None
    for k, v in config.items():
        try:
            if k.lower() == name.lower():
                cfg = v
                return cfg
            else:
                try:
                    cfg = find_cfg(v, name)
                    if not cfg == None:
                        return cfg
                except Exception as exc:
                    raise exc
        except AttributeError:
            pass
        except Exception as exc:
            raise exc
    return cfg

class GroupWidget(QWidget):
    """
    (Isaac's guess as of 12/13/2012) 
    This class bonds multiple Editor instances that are associated with
    a single node as a group. 
    """

    def __init__(self, updater, config):
        """
        :param config: defined in dynamic_reconfigure.client.Client
        :type config: Dictionary?
        """

        super(GroupWidget, self).__init__()
        self.state = config['state']
        self.name = config['name']

        self.tab_bar = None  # Every group can have one tab bar
        self.tab_bar_shown = False

        #self.grid = QGridLayout()
        self.grid = QFormLayout()

        self.updater = updater

        self.editor_widgets = []
        self.add_widgets(config)

        # Labels should not stretch
        #self.grid.setColumnStretch(1, 1)
        self.setLayout(self.grid)

    def add_widgets(self, config):
        """
        :type config: Dict?
        """
        i_debug = 0
        for param in config['parameters']:
            if param['edit_method']:
                widget = EnumEditor(self.updater, param)
            elif param['type'] in EDITOR_TYPES:
                widget = eval(EDITOR_TYPES[param['type']])(self.updater, param)

            self.editor_widgets.append(widget)
            rospy.logdebug('groups.add_widgets num editors=%d', i_debug)
            i_debug += 1

        g_debug = 0
        for name, group in config['groups'].items():
            if group['type'] == 'tab':
                widget = TabGroup(self, self.updater, group)
            elif group['type'] in _GROUP_TYPES.keys():
                widget = eval(_GROUP_TYPES[group['type']])(self.updater, group)

            self.editor_widgets.append(widget)
            rospy.logdebug('groups.add_widgets num groups=%d', g_debug)
            g_debug += 1

        for i, ed in enumerate(self.editor_widgets):
            ed.display(self.grid, i)

        rospy.logdebug('GroupWidget.add_widgets len(self.editor_widgets)=%d',
                      len(self.editor_widgets))

    def display(self, grid, row):
        # groups span across all columns
        grid.addWidget(self, row, 0, 1, -1)

    def update_group(self, config):
        self.state = config['state']

        # TODO: should use config.keys but this method doesnt exist
        names = [name for name, v in config.items()]

        for widget in self.editor_widgets:
            if isinstance(widget, EditorWidget):
                if widget.name in names:
                    widget.update_value(config[widget.name])
            elif isinstance(widget, GroupWidget):
                cfg = find_cfg(config, widget.name)
                widget.update_group(cfg)

    def close(self):
        for w in self.editor_widgets:
            w.close()

class BoxGroup(GroupWidget):
    def __init__(self, updater, config):
        super(BoxGroup, self).__init__(updater, config)

        self.box = QGroupBox(self.name)
        self.box.setLayout(self.grid)

    def display(self, grid, row):
        grid.addWidget(self.box, row, 0, 1, -1)

class CollapseGroup(BoxGroup):
    def __init__(self, updater, config):
        super(CollapseGroup, self).__init__(updater, config)
        self.box.setCheckable(True)

class HideGroup(BoxGroup):
    def update_group(self, config):
        super(HideGroup, self).update_group(config)
        self.box.setVisible(self.state)

class TabGroup(GroupWidget):
    def __init__(self, parent, updater, config):
        super(TabGroup, self).__init__(updater, config)
        self.parent = parent

        if self.parent.tab_bar is None:
            self.parent.tab_bar = QTabWidget()

        parent.tab_bar.addTab(self, self.name)

    def display(self, grid, row):
        if not self.parent.tab_bar_shown:
            grid.addWidget(self.parent.tab_bar, row, 0, 1, -1)
            self.parent.tab_bar_shown = True

    def close(self):
        super(TabGroup, self).close()
        self.parent.tab_bar = None
        self.parent.tab_bar_shown = False

class ApplyGroup(BoxGroup):
    class ApplyUpdater:
        def __init__(self, updater):
            self.updater = updater
            self._pending_config = {}

        def update(self, config):
            for name, value in config.items():
                self._pending_config[name] = value

        def apply_update(self):
            self.updater.update(self._pending_config)
            self._pending_config = {}

    def __init__(self, updater, config):
        self.updater = ApplyGroup.ApplyUpdater(updater)
        super(ApplyGroup, self).__init__(self.updater, config)

        self.button = QPushButton("Apply %s" % self.name)
        self.button.clicked.connect(self.updater.apply_update)

        rows = self.grid.rowCount()
        self.grid.addWidget(self.button, rows + 1, 1, Qt.AlignRight)
