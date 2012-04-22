# Copyright (c) 2011, Dirk Thomas, TU Darmstadt
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

from __future__ import division
import os

# pydot requires some hacks
import pydot
# TODO: use pygraphviz instead, but non-deterministic layout will first be resolved in graphviz 2.30
#import pygraphviz

from rosgui.QtBindingHelper import loadUi
from QtCore import QEvent, QFile, QIODevice, QObject, QPointF, QRectF, Qt, QTextStream, Signal
from QtGui import QColor, QFileDialog, QGraphicsScene, QIcon, QImage, QPainter, QWidget
from QtSvg import QSvgGenerator

import roslib
roslib.load_manifest('rosgui_rosgraph')
import rosgraph.impl.graph, rostopic, rosnode, rosservice

import dotcode
reload(dotcode)
from dotcode import generate_dotcode, NODE_NODE_GRAPH, NODE_TOPIC_ALL_GRAPH, NODE_TOPIC_GRAPH

import rosgui_dotgraph.edge_item
reload(rosgui_dotgraph.edge_item)
from rosgui_dotgraph.edge_item import EdgeItem
import InteractiveGraphicsView
reload(InteractiveGraphicsView)
from InteractiveGraphicsView import InteractiveGraphicsView
import rosgui_dotgraph.node_item
reload(rosgui_dotgraph.node_item)
from rosgui_dotgraph.node_item import NodeItem

class RosGraph(QObject):

    _deferred_fit_in_view = Signal()

    def __init__(self, context):
        super(RosGraph, self).__init__(context)
        self.setObjectName('RosGraph')

        self._graph = None
        self._current_dotcode = None

        self._widget = QWidget()

        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'RosGraph.ui')
        loadUi(ui_file, self._widget, {'InteractiveGraphicsView': InteractiveGraphicsView})
        self._widget.setObjectName('RosGraphUi')
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        self._scene = QGraphicsScene()
        self._widget.graphics_view.setScene(self._scene)

        self._widget.graph_type_combo_box.insertItem(0, self.tr('Nodes only'), NODE_NODE_GRAPH)
        self._widget.graph_type_combo_box.insertItem(1, self.tr('Nodes/Topics (active)'), NODE_TOPIC_GRAPH)
        self._widget.graph_type_combo_box.insertItem(2, self.tr('Nodes/Topics (all)'), NODE_TOPIC_ALL_GRAPH)
        self._widget.graph_type_combo_box.setCurrentIndex(0)
        self._widget.graph_type_combo_box.currentIndexChanged.connect(self._refresh_rosgraph)
        self._widget.filter_line_edit.editingFinished.connect(self._refresh_rosgraph)
        self._widget.quiet_check_box.clicked.connect(self._refresh_rosgraph)

        self._widget.refresh_graph_push_button.setIcon(QIcon.fromTheme('view-refresh'))
        self._widget.refresh_graph_push_button.pressed.connect(self._update_rosgraph)

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

        self._update_rosgraph()
        self._deferred_fit_in_view.connect(self._fit_in_view, Qt.QueuedConnection)
        self._deferred_fit_in_view.emit()

        context.add_widget(self._widget)

    def save_settings(self, global_settings, perspective_settings):
        perspective_settings.set_value('graph_type_combo_box_index', self._widget.graph_type_combo_box.currentIndex())
        perspective_settings.set_value('filter_line_edit_text', self._widget.filter_line_edit.text())
        perspective_settings.set_value('quiet_check_box_state', self._widget.quiet_check_box.isChecked())
        perspective_settings.set_value('auto_fit_graph_check_box_state', self._widget.auto_fit_graph_check_box.isChecked())
        perspective_settings.set_value('highlight_connections_check_box_state', self._widget.highlight_connections_check_box.isChecked())

    def restore_settings(self, global_settings, perspective_settings):
        self._widget.graph_type_combo_box.setCurrentIndex(int(perspective_settings.value('graph_type_combo_box_index', 0)))
        self._widget.filter_line_edit.setText(perspective_settings.value('filter_line_edit_text', '/'))
        self._widget.quiet_check_box.setChecked(perspective_settings.value('quiet_check_box_state', True) in [True, 'true'])
        self._widget.auto_fit_graph_check_box.setChecked(perspective_settings.value('auto_fit_graph_check_box_state', True) in [True, 'true'])
        self._widget.highlight_connections_check_box.setChecked(perspective_settings.value('highlight_connections_check_box_state', True) in [True, 'true'])
        self._refresh_rosgraph()

    def _update_rosgraph(self):
        # re-enable controls customizing fetched ROS graph
        self._widget.graph_type_combo_box.setEnabled(True)
        self._widget.filter_line_edit.setEnabled(True)
        self._widget.quiet_check_box.setEnabled(True)

        self._graph = rosgraph.impl.graph.Graph()
        self._graph.set_master_stale(5.0)
        self._graph.set_node_stale(5.0)
        self._graph.update()
        self._refresh_rosgraph()

    def _refresh_rosgraph(self):
        self._update_graph_view(self._generate_dotcode())

    def _generate_dotcode(self):
        ns_filter = self._widget.filter_line_edit.text()
        if not ns_filter.endswith('/'):
            ns_filter += '/'
        graph_mode = self._widget.graph_type_combo_box.itemData(self._widget.graph_type_combo_box.currentIndex())
        orientation = 'LR'
        quiet = self._widget.quiet_check_box.isChecked()
        return generate_dotcode(self._graph, ns_filter, graph_mode, orientation, quiet)

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

        # read dot graph
        raw_graph = pydot.graph_from_dot_data(self._current_dotcode)
        # layout graph
        graph = pydot.graph_from_dot_data(raw_graph.create_dot('dot'))

        #graph = pygraphviz.AGraph(string=self._current_dotcode, strict=False, directed=True)
        #graph.layout(prog='dot')

        POINTS_PER_INCH = 72

        # hack required by pydot
        def get_unquoted(item, name):
            value = item.get(name)
            if value.startswith('"') and value.endswith('"'):
                value = value[1:-1]
            value = value.replace('\\\n', '')
            return value
        # let pydot imitate pygraphviz api
        graph.nodes_iter = graph.get_node_list
        graph.edges_iter = graph.get_edge_list

        nodes = {}
        for node in graph.nodes_iter():
            # hack required by pydot
            if node.get_name() in ('graph', 'node', 'empty'):
                continue
            # let pydot imitate pygraphviz api
            attr = {}
            for name in node.get_attributes().iterkeys():
                value = get_unquoted(node, name)
                attr[name] = value
            node.attr = attr

            # decrease rect by one so that edges do not reach inside
            bounding_box = QRectF(0, 0, POINTS_PER_INCH * float(node.attr['width']) - 1.0, POINTS_PER_INCH * float(node.attr['height']) - 1.0)
            pos = node.attr['pos'].split(',')
            bounding_box.moveCenter(QPointF(float(pos[0]), -float(pos[1])))
            color = QColor(node.attr['color']) if 'color' in node.attr else None
            node_item = NodeItem(highlight_level, bounding_box, node.attr['label'], node.attr.get('shape', 'ellipse'), color)
            node_item.setToolTip(self._generate_tool_tip(node.attr.get('URL', None)))

            # let pydot imitate pygraphviz api
            pydot.Node.__repr__ = lambda self: self.get_name()

            nodes[str(node)] = node_item

        edges = {}
        for edge in graph.edges_iter():
            # let pydot imitate pygraphviz api
            attr = {}
            for name in edge.get_attributes().iterkeys():
                value = get_unquoted(edge, name)
                attr[name] = value
            edge.attr = attr

            label = edge.attr.get('label', None)
            label_pos = edge.attr.get('lp', None)
            label_center = None
            if label_pos is not None:
                label_pos = label_pos.split(',')
                label_center = QPointF(float(label_pos[0]), -float(label_pos[1]))

            # try pydot, fallback for pygraphviz
            source_node = edge.get_source() if hasattr(edge, 'get_source') else edge[0]
            destination_node = edge.get_destination() if hasattr(edge, 'get_destination') else edge[1]

            # create edge with from-node and to-node
            edge_item = EdgeItem(highlight_level, edge.attr['pos'], label_center, label, nodes[source_node], nodes[destination_node])
            # symmetrically add all sibling edges with same label
            if label is not None:
                if label not in edges:
                    edges[label] = []
                for sibling in edges[label]:
                    edge_item.add_sibling_edge(sibling)
                    sibling.add_sibling_edge(edge_item)
                edges[label].append(edge_item)

            edge_item.setToolTip(self._generate_tool_tip(edge.attr.get('URL', None)))
            edge_item.add_to_scene(self._scene)

        for node_item in nodes.itervalues():
            self._scene.addItem(node_item)

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
        self._widget.graph_type_combo_box.setEnabled(False)
        self._widget.filter_line_edit.setEnabled(False)
        self._widget.quiet_check_box.setEnabled(False)

        self._update_graph_view(dotcode)

    def _fit_in_view(self):
        self._widget.graphics_view.fitInView(self._scene.itemsBoundingRect(), Qt.KeepAspectRatio)

    def _save_dot(self):
        file_name, _ = QFileDialog.getSaveFileName(self._widget, self.tr('Save as DOT'), 'rosgraph.dot', self.tr('DOT graph (*.dot)'))
        if file_name is None or file_name == '':
            return

        file = QFile(file_name)
        if not file.open(QIODevice.WriteOnly | QIODevice.Text):
            return

        file.write(self._current_dotcode)
        file.close()

    def _save_svg(self):
        file_name, _ = QFileDialog.getSaveFileName(self._widget, self.tr('Save as SVG'), 'rosgraph.svg', self.tr('Scalable Vector Graphic (*.svg)'))
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
        file_name, _ = QFileDialog.getSaveFileName(self._widget, self.tr('Save as image'), 'rosgraph.png', self.tr('Image (*.bmp *.jpg *.png *.tiff)'))
        if file_name is None or file_name == '':
            return

        img = QImage((self._scene.sceneRect().size() * 2.0).toSize(), QImage.Format_ARGB32_Premultiplied)
        painter = QPainter(img)
        painter.setRenderHint(QPainter.Antialiasing)
        self._scene.render(painter)
        painter.end()
        img.save(file_name)
