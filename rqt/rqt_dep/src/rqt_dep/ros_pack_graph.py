# Copyright (c) 2011, Thibault Kruse
# All rights reserved.

from __future__ import division
import os

import rospkg

from qt_gui.qt_binding_helper import loadUi
from QtCore import QFile, QIODevice, QObject, Qt, Signal, QAbstractListModel
from QtGui import QFileDialog, QGraphicsScene, QIcon, QImage, QPainter, QWidget, QCompleter
from QtSvg import QSvgGenerator

import roslib
roslib.load_manifest('rqt_dep')
import rosservice
import rostopic

from .dotcode_pack import RosPackageGraphDotcodeGenerator
from qt_dotgraph.pydotfactory import PydotFactory
# from qt_dotgraph.pygraphvizfactory import PygraphvizFactory
from qt_dotgraph.dot_to_qt import DotToQtGenerator

from rqt_gui_py.plugin import Plugin
from rqt_graph.interactive_graphics_view import InteractiveGraphicsView


class RepeatedWordCompleter(QCompleter):
    """A completer that completes multiple times from a list"""
    def pathFromIndex(self, index):
        path = QCompleter.pathFromIndex(self, index)
        lst = str(self.widget().text()).split(',')
        if len(lst) > 1:
            path = '%s, %s' % (','.join(lst[:-1]), path)
        return path

    def splitPath(self, path):
        path = str(path.split(',')[-1]).lstrip(' ')
        return [path]


class StackageCompletionModel(QAbstractListModel):
    """Ros package and stacknames"""
    def __init__(self, linewidget, rospack, rosstack):
        super(QAbstractListModel, self).__init__(linewidget)
        self.allnames = sorted(list(set(rospack.list() + rosstack.list())))
        self.allnames = self.allnames + ['-%s' % name for name in self.allnames]

    def rowCount(self, parent):
        return len(self.allnames)

    def data(self, index, role):
        # TODO: symbols to distinguish stacks from packages
        if index.isValid() and (role == Qt.DisplayRole or role == Qt.EditRole):
            return self.allnames[index.row()]
        return None


class RosPackGraph(Plugin):

    _deferred_fit_in_view = Signal()

    def __init__(self, context):
        super(RosPackGraph, self).__init__(context)
        self.initialized = False

        self.setObjectName('RosPackGraph')

        self._current_dotcode = None

        self._widget = QWidget()

        rospack = rospkg.RosPack()
        rosstack = rospkg.RosStack()

        # factory builds generic dotcode items
        self.dotcode_factory = PydotFactory()
        # self.dotcode_factory = PygraphvizFactory()
        # generator builds rosgraph
        self.dotcode_generator = RosPackageGraphDotcodeGenerator(rospack, rosstack)
        # dot_to_qt transforms into Qt elements using dot layout
        self.dot_to_qt = DotToQtGenerator()

        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'RosPackGraph.ui')
        loadUi(ui_file, self._widget, {'InteractiveGraphicsView': InteractiveGraphicsView})
        self._widget.setObjectName('RosPackGraphUi')
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        self._scene = QGraphicsScene()
        self._widget.graphics_view.setScene(self._scene)

        self._widget.depth_combo_box.insertItem(0, self.tr('infinite'), -1)
        self._widget.depth_combo_box.insertItem(1, self.tr('1'), 2)
        self._widget.depth_combo_box.insertItem(2, self.tr('2'), 3)
        self._widget.depth_combo_box.insertItem(3, self.tr('3'), 4)
        self._widget.depth_combo_box.insertItem(4, self.tr('4'), 5)
        self._widget.depth_combo_box.setCurrentIndex(0)
        self._widget.depth_combo_box.currentIndexChanged.connect(self._refresh_rospackgraph)

        self._widget.directions_combo_box.insertItem(0, self.tr('depends'), 0)
        self._widget.directions_combo_box.insertItem(1, self.tr('depends_on'), 1)
        self._widget.directions_combo_box.insertItem(2, self.tr('both'), 2)
        self._widget.directions_combo_box.setCurrentIndex(2)
        self._widget.directions_combo_box.currentIndexChanged.connect(self._refresh_rospackgraph)

        completionmodel = StackageCompletionModel(self._widget.filter_line_edit, rospack, rosstack)
        completer = RepeatedWordCompleter(completionmodel, self)
        completer.setCompletionMode(QCompleter.PopupCompletion)
        completer.setWrapAround(True)

        completer.setCaseSensitivity(Qt.CaseInsensitive)
        self._widget.filter_line_edit.editingFinished.connect(self._refresh_rospackgraph)
        self._widget.filter_line_edit.setCompleter(completer)

        self._widget.with_stacks_check_box.clicked.connect(self._refresh_rospackgraph)
        self._widget.mark_check_box.clicked.connect(self._refresh_rospackgraph)
        self._widget.colorize_check_box.clicked.connect(self._refresh_rospackgraph)
        self._widget.transitives_check_box.clicked.connect(self._refresh_rospackgraph)

        self._widget.refresh_graph_push_button.setIcon(QIcon.fromTheme('view-refresh'))
        self._widget.refresh_graph_push_button.pressed.connect(self._update_rospackgraph)

        self._widget.highlight_connections_check_box.toggled.connect(self._redraw_graph_view)
        self._widget.auto_fit_graph_check_box.toggled.connect(self._redraw_graph_view)
        self._widget.fit_in_view_push_button.setIcon(QIcon.fromTheme('zoom-original'))
        self._widget.fit_in_view_push_button.pressed.connect(self._fit_in_view)

        self._widget.load_dot_push_button.setIcon(QIcon.fromTheme('document-open'))
        self._widget.load_dot_push_button.pressed.connect(self._load_dot)
        self._widget.save_dot_push_button.setIcon(QIcon.fromTheme('document-save-as'))
        self._widget.save_dot_push_button.pressed.connect(self._save_dot)
        self._widget.save_as_svg_push_button.setIcon(QIcon.fromTheme('document-save-as'))
        self._widget.save_as_svg_push_button.pressed.connect(self._save_svg)
        self._widget.save_as_image_push_button.setIcon(QIcon.fromTheme('image'))
        self._widget.save_as_image_push_button.pressed.connect(self._save_image)

        self._deferred_fit_in_view.connect(self._fit_in_view, Qt.QueuedConnection)
        self._deferred_fit_in_view.emit()

        context.add_widget(self._widget)

    def save_settings(self, plugin_settings, instance_settings):
        instance_settings.set_value('depth_combo_box_index', self._widget.depth_combo_box.currentIndex())
        instance_settings.set_value('directions_combo_box_index', self._widget.directions_combo_box.currentIndex())
        instance_settings.set_value('filter_line_edit_text', self._widget.filter_line_edit.text())
        instance_settings.set_value('with_stacks_state', self._widget.with_stacks_check_box.isChecked())
        instance_settings.set_value('transitives_state', self._widget.transitives_check_box.isChecked())
        instance_settings.set_value('mark_state', self._widget.mark_check_box.isChecked())
        instance_settings.set_value('colorize_state', self._widget.colorize_check_box.isChecked())
        instance_settings.set_value('auto_fit_graph_check_box_state', self._widget.auto_fit_graph_check_box.isChecked())
        instance_settings.set_value('highlight_connections_check_box_state', self._widget.highlight_connections_check_box.isChecked())

    def restore_settings(self, plugin_settings, instance_settings):
        self._widget.depth_combo_box.setCurrentIndex(int(instance_settings.value('depth_combo_box_index', 0)))
        self._widget.directions_combo_box.setCurrentIndex(int(instance_settings.value('directions_combo_box_index', 0)))
        self._widget.filter_line_edit.setText(instance_settings.value('filter_line_edit_text', ''))
        self._widget.with_stacks_check_box.setChecked(instance_settings.value('with_stacks_state', True) in [True, 'true'])
        self._widget.mark_check_box.setChecked(instance_settings.value('mark_state', True) in [True, 'true'])
        self._widget.colorize_check_box.setChecked(instance_settings.value('colorize_state', True) in [True, 'true'])
        self._widget.transitives_check_box.setChecked(instance_settings.value('transitives_state', True) in [True, 'true'])
        self._widget.auto_fit_graph_check_box.setChecked(instance_settings.value('auto_fit_graph_check_box_state', True) in [True, 'true'])
        self._widget.highlight_connections_check_box.setChecked(instance_settings.value('highlight_connections_check_box_state', True) in [True, 'true'])
        self.initialized = True
        self._refresh_rospackgraph()

    def _update_rospackgraph(self):
        # re-enable controls customizing fetched ROS graph
        self._widget.depth_combo_box.setEnabled(True)
        self._widget.directions_combo_box.setEnabled(True)
        self._widget.filter_line_edit.setEnabled(True)
        self._widget.with_stacks_check_box.setEnabled(True)
        self._widget.mark_check_box.setEnabled(True)
        self._widget.colorize_check_box.setEnabled(True)
        self._widget.transitives_check_box.setEnabled(True)

        self._refresh_rospackgraph()

    def _refresh_rospackgraph(self):
        if not self.initialized:
            return
        self._update_graph_view(self._generate_dotcode())

    def _generate_dotcode(self):
        names = self._widget.filter_line_edit.text().split(',')
        if names == [u'None']:
            names = []
        includes = []
        excludes = []
        for name in names:
            if name.strip().startswith('-'):
                excludes.append(name.strip()[1:])
            else:
                includes.append(name.strip())
        depth = self._widget.depth_combo_box.itemData(self._widget.depth_combo_box.currentIndex())
        # orientation = 'LR'
        descendants = True
        ancestors = True
        selected_directions = self._widget.directions_combo_box.itemData(self._widget.directions_combo_box.currentIndex())
        if selected_directions == 1:
            descendants = False
        if selected_directions == 0:
            ancestors = False
        # TODO: Allow different color themes
        colortheme = None
        if self._widget.colorize_check_box.isChecked():
            colortheme = True
        return self.dotcode_generator.generate_dotcode(dotcode_factory=self.dotcode_factory,
                                                       selected_names=includes,
                                                       excludes=excludes,
                                                       depth=depth,
                                                       with_stacks=self._widget.with_stacks_check_box.isChecked(),
                                                       descendants=descendants,
                                                       ancestors=ancestors,
                                                       mark_selected=self._widget.mark_check_box.isChecked(),
                                                       colortheme=colortheme,
                                                       hide_transitives=self._widget.transitives_check_box.isChecked())

    def _update_graph_view(self, dotcode):
        if dotcode == self._current_dotcode:
            return
        self._current_dotcode = dotcode
        self._redraw_graph_view()

    def _generate_tool_tip(self, url):
        if url is not None and ':' in url:
            item_type, item_path = url.split(':', 1)
            if item_type == 'node':
                tool_tip = 'Node:\n  %s' % (item_path)
                service_names = rosservice.get_service_list(node=item_path)
                if service_names:
                    tool_tip += '\nServices:'
                    for service_name in service_names:
                        try:
                            service_type = rosservice.get_service_type(service_name)
                            tool_tip += '\n  %s [%s]' % (service_name, service_type)
                        except rosservice.ROSServiceIOException, e:
                            tool_tip += '\n  %s' % (e)
                return tool_tip
            elif item_type == 'topic':
                topic_type, topic_name, _ = rostopic.get_topic_type(item_path)
                return 'Topic:\n  %s\nType:\n  %s' % (topic_name, topic_type)
        return url

    def _redraw_graph_view(self):
        self._scene.clear()

        if self._widget.highlight_connections_check_box.isChecked():
            highlight_level = 3
        else:
            highlight_level = 1

        (nodes, edges) = self.dot_to_qt.dotcode_to_qt_items(self._current_dotcode, highlight_level)

        for node_item in nodes.itervalues():
            self._scene.addItem(node_item)
        for edge_items in edges.itervalues():
            for edge_item in edge_items:
                edge_item.add_to_scene(self._scene)

        self._scene.setSceneRect(self._scene.itemsBoundingRect())
        if self._widget.auto_fit_graph_check_box.isChecked():
            self._fit_in_view()

    def _load_dot(self, file_name=None):
        if file_name is None:
            file_name, _ = QFileDialog.getOpenFileName(self._widget, self.tr('Open graph from file'), None, self.tr('DOT graph (*.dot)'))
            if file_name is None or file_name == '':
                return

        try:
            fh = open(file_name, 'rb')
            dotcode = fh.read()
            fh.close()
        except IOError:
            return

        # disable controls customizing fetched ROS graph
        self._widget.depth_combo_box.setEnabled(False)
        self._widget.directions_combo_box.setEnabled(False)
        self._widget.filter_line_edit.setEnabled(False)
        self._widget.with_stacks_check_box.setEnabled(False)
        self._widget.mark_check_box.setEnabled(False)
        self._widget.colorize_check_box.setEnabled(False)
        self._widget.transitives_check_box.setEnabled(False)

        self._update_graph_view(dotcode)

    def _fit_in_view(self):
        self._widget.graphics_view.fitInView(self._scene.itemsBoundingRect(), Qt.KeepAspectRatio)

    def _save_dot(self):
        file_name, _ = QFileDialog.getSaveFileName(self._widget, self.tr('Save as DOT'), 'rospackgraph.dot', self.tr('DOT graph (*.dot)'))
        if file_name is None or file_name == '':
            return

        handle = QFile(file_name)
        if not handle.open(QIODevice.WriteOnly | QIODevice.Text):
            return

        handle.write(self._current_dotcode)
        handle.close()

    def _save_svg(self):
        file_name, _ = QFileDialog.getSaveFileName(self._widget, self.tr('Save as SVG'), 'rospackgraph.svg', self.tr('Scalable Vector Graphic (*.svg)'))
        if file_name is None or file_name == '':
            return

        generator = QSvgGenerator()
        generator.setFileName(file_name)
        generator.setSize((self._scene.sceneRect().size() * 2.0).toSize())

        painter = QPainter(generator)
        painter.setRenderHint(QPainter.Antialiasing)
        self._scene.render(painter)
        painter.end()

    def _save_image(self):
        file_name, _ = QFileDialog.getSaveFileName(self._widget, self.tr('Save as image'), 'rospackgraph.png', self.tr('Image (*.bmp *.jpg *.png *.tiff)'))
        if file_name is None or file_name == '':
            return

        img = QImage((self._scene.sceneRect().size() * 2.0).toSize(), QImage.Format_ARGB32_Premultiplied)
        painter = QPainter(img)
        painter.setRenderHint(QPainter.Antialiasing)
        self._scene.render(painter)
        painter.end()
        img.save(file_name)
