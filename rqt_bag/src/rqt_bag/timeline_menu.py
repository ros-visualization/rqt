# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
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

from python_qt_binding.QtGui import QVBoxLayout, QMenu, QWidget


class TimelinePopupMenu(QMenu):
    """
    Custom popup menu displayed on rightclick from timeline
    """
    def __init__(self, timeline, event):
        super(TimelinePopupMenu, self).__init__()

        self.parent = timeline
        self.timeline = timeline

        self._reset_timeline = self.addAction('Reset Timeline')

        self._play_all = self.addAction('Play All Messages')
        self._play_all.setCheckable(True)
        self._play_all.setChecked(self.timeline.play_all)

        self.addSeparator()

        submenu = self.addMenu('Thumbnails...')
        self._thumbnail_show_action = submenu.addAction('Show All')
        self._thumbnail_hide_action = submenu.addAction('Hide All')
        submenu.addSeparator()

        self._renderers = self.timeline._timeline_frame.get_renderers()
        self._thumbnail_actions = []
        for topic, renderer in self._renderers:
            self._thumbnail_actions.append(submenu.addAction(topic))
            self._thumbnail_actions[-1].setCheckable(True)
            self._thumbnail_actions[-1].setChecked(self.timeline._timeline_frame.is_renderer_active(topic))

        self._topics = self.timeline._timeline_frame.topics
        view_topics_menu = self.addMenu('View (by Topic)')
        self._topic_actions = []
        for topic in self._topics:
            datatype = self.timeline.get_datatype(topic)

            # View... / topic
            topic_menu = QMenu(topic, self)
            viewer_types = self.timeline._timeline_frame.get_viewer_types(datatype)

            # View... / topic / Viewer
            for viewer_type in viewer_types:
                tempaction = topic_menu.addAction(viewer_type.name)
                tempaction.setData(viewer_type)
                self._topic_actions.append(tempaction)
            view_topics_menu.addMenu(topic_menu)

        view_type_menu = self.addMenu('View (by Type)')
        self._topics_by_type = self.timeline._timeline_frame._topics_by_datatype
        self._type_actions = []
        for datatype in self._topics_by_type:
            # View... / datatype
            datatype_menu = QMenu(datatype, self)
            datatype_topics = self._topics_by_type[datatype]
            viewer_types = self.timeline._timeline_frame.get_viewer_types(datatype)
            for topic in [t for t in self._topics if t in datatype_topics]:   # use timeline ordering
                topic_menu = QMenu(topic, datatype_menu)
                # View... / datatype / topic / Viewer
                for viewer_type in viewer_types:
                    tempaction = topic_menu.addAction(viewer_type.name)
                    tempaction.setData(viewer_type)
                    self._topic_actions.append(tempaction)
                datatype_menu.addMenu(topic_menu)
            view_type_menu.addMenu(datatype_menu)

        self.addSeparator()
        submenu = self.addMenu('Publish...')

        self._publish_all = submenu.addAction('Publish All')
        self._publish_none = submenu.addAction('Publish None')
        submenu.addSeparator()

        self._publish_actions = []
        for topic in self._topics:
            self._publish_actions.append(submenu.addAction(topic))
            self._publish_actions[-1].setCheckable(True)
            self._publish_actions[-1].setChecked(self.timeline.is_publishing(topic))

        action = self.exec_(event.globalPos())
        if action is not None and action != 0:
            self.process(action)

    def process(self, action):
        """
        :param action: action to execute, ''QAction''
        :raises: when it doesn't recognice the action passed in, ''Exception''
        """
        if action == self._reset_timeline:
            self.timeline._timeline_frame.reset_timeline()
        elif action == self._play_all:
            self.timeline.toggle_play_all()
        elif action == self._publish_all:
            for topic in self.timeline._timeline_frame.topics:
                if not self.timeline.start_publishing(topic):
                    break
        elif action == self._publish_none:
            for topic in self.timeline._timeline_frame.topics:
                if not self.timeline.stop_publishing(topic):
                    break
        elif action == self._thumbnail_show_action:
            self.timeline._timeline_frame.set_renderers_active(True)
        elif action == self._thumbnail_hide_action:
            self.timeline._timeline_frame.set_renderers_active(False)
        elif action in self._thumbnail_actions:
            if self.timeline._timeline_frame.is_renderer_active(action.text()):
                self.timeline._timeline_frame.set_renderer_active(action.text(), False)
            else:
                self.timeline._timeline_frame.set_renderer_active(action.text(), True)
        elif action in self._topic_actions + self._type_actions:
            popup_name = action.parentWidget().title() + '__' + action.text()
            if popup_name not in self.timeline.popups:
                frame = QWidget()
                layout = QVBoxLayout()
                frame.setLayout(layout)
                frame.resize(640, 480)
                viewer_type = action.data()
                frame.setObjectName(popup_name)
                view = viewer_type(self.timeline, frame)
                self.timeline.popups.add(popup_name)
                self.timeline.get_context().add_widget(frame)
                self.timeline.add_view(action.parentWidget().title(), view, frame)
                frame.show()
        elif action in self._publish_actions:
            if self.timeline.is_publishing(action.text()):
                self.timeline.stop_publishing(action.text())
            else:
                self.timeline.start_publishing(action.text())
        else:
            raise Exception('Unknown action in TimelinePopupMenu.process')
