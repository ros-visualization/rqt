# Copyright (c) 2011, Thibault Kruse
# All rights reserved.

from __future__ import division
import os
import re

# pydot requires some hacks
import pydot

import rospkg

from rosgui.QtBindingHelper import loadUi
from QtCore import QEvent, QFile, QIODevice, QObject, QPointF, QRectF, Qt, QTextStream, Signal, QAbstractListModel, SIGNAL
from QtGui import QColor, QFileDialog, QGraphicsScene, QIcon, QImage, QPainter, QWidget, QCompleter
from QtSvg import QSvgGenerator

import roslib
roslib.load_manifest('rosgui_package_graph')
import rosgraph.impl.graph

import dotcode_pack
reload(dotcode_pack)
from dotcode_pack import generate_dotcode

import EdgeItem
reload(EdgeItem)
from EdgeItem import EdgeItem

import InteractiveGraphicsView
reload(InteractiveGraphicsView)
from InteractiveGraphicsView import InteractiveGraphicsView

import NodeItem
reload(NodeItem)
from NodeItem import NodeItem

POINTS_PER_INCH = 72

class RepeatedWordCompleter(QCompleter):
    """A completer that completes multiple times from a list"""
    def init(self, parent=None):
        QCompleter.init(self, parent)

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
    def __init__(self, linewidget):
        super(QAbstractListModel, self).__init__(linewidget)
        self.allnames = sorted(list(set(rospkg.RosPack().list() + rospkg.RosStack().list())))
        self.allnames = self.allnames + ['-%s'%name for name in self.allnames]
    def rowCount(self, parent):
        return len(self.allnames)
    
    def data(self, index, role):
        # TODO: symbols to distinguish stacks from packages
        if index.isValid() and (role == Qt.DisplayRole or role == Qt.EditRole):
            return self.allnames[index.row()]
        return None

# hack required by pydot
def get_unquoted(item, name):
    value = item.get(name)
    return value.strip('"\n"')

def getNodeItemForStack(subgraph, highlight_level):
    # let pydot imitate pygraphviz api
    attr = {}
    for name in subgraph.get_attributes().iterkeys():
        value = get_unquoted(node, name)
        attr[name] = value
    obj_dic = subgraph.__getattribute__("obj_dict")
    for name in obj_dic:
        if name not in ['nodes', 'attributes', 'parent_graph'] and obj_dic[name] is not None:
            attr[name] = obj_dic[name]
        elif name == 'nodes':
            for key in obj_dic['nodes']['graph'][0]['attributes']:
                attr[key] = obj_dic['nodes']['graph'][0]['attributes'][key]
    subgraph.attr = attr

    bb = subgraph.attr['bb'].strip('"').split(',')
    bounding_box = QRectF(0,0, float(bb[2]) - float(bb[0]), float(bb[3]) -float(bb[1]))
    label_pos = subgraph.attr['lp'].strip('"').split(',')
    bounding_box.moveCenter(QPointF(float(bb[0]) + (float(bb[2]) - float(bb[0])) / 2, - float(bb[1]) - (float(bb[3]) -float(bb[1]))/2))
    name = subgraph.attr['label']
    stacknodeitem = NodeItem(highlight_level, bounding_box, name, 'box', color=None, label_pos=QPointF(float(label_pos[0]), -float(label_pos[1])))
    bounding_box = QRectF(bounding_box)
    bounding_box.setHeight(30)
    stacknodeitem.set_hovershape(bounding_box)
    return stacknodeitem

def getNodeItemForNode(node, highlight_level):
# let pydot imitate pygraphviz api
    attr = {}
    for name in node.get_attributes().iterkeys():
        value = get_unquoted(node, name)
        attr[name] = value
    obj_dic = node.__getattribute__("obj_dict")
    for name in obj_dic:
        if name not in ['attributes', 'parent_graph'] and obj_dic[name] is not None:
            attr[name] = obj_dic[name]
    node.attr = attr
    
    # decrease rect by one so that edges do not reach inside
    bounding_box = QRectF(0, 0, POINTS_PER_INCH * float(node.attr['width']) - 1.0, POINTS_PER_INCH * float(node.attr['height']) - 1.0)
    pos = node.attr['pos'].split(',')
    bounding_box.moveCenter(QPointF(float(pos[0]), -float(pos[1])))
    color = QColor(node.attr['color']) if 'color' in node.attr else None
    name = node.attr['label']
    if name is None:
        # happens on Lucid version
        print("Error, label is None for node %s, pygraphviz version may be too old."%node)
    node_item = NodeItem(highlight_level, bounding_box, name, node.attr.get('shape', 'ellipse'), color)
    #node_item.setToolTip(self._generate_tool_tip(node.attr.get('URL', None)))
    return node_item

def getEdgeItem(edge, nodes, edges, highlight_level):
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
    return edge_item


class RosPackGraph(QObject):

    _deferred_fit_in_view = Signal()

    def __init__(self, context):
        super(RosPackGraph, self).__init__(context)
        self.setObjectName('RosPackGraph')

        self._current_dotcode = None

        self._widget = QWidget()

        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'RosPackGraph.ui')
        loadUi(ui_file, self._widget, {'InteractiveGraphicsView': InteractiveGraphicsView})
        self._widget.setObjectName('RosPackGraphUi')
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        self._scene = QGraphicsScene()
        self._widget.graphics_view.setScene(self._scene)

        self._widget.graph_type_combo_box.insertItem(0, self.tr('infinite'), -1)
        self._widget.graph_type_combo_box.insertItem(1, self.tr('1'), 2)
        self._widget.graph_type_combo_box.insertItem(2, self.tr('2'), 3)
        self._widget.graph_type_combo_box.insertItem(3, self.tr('3'), 4)
        self._widget.graph_type_combo_box.insertItem(4, self.tr('4'), 5)
        self._widget.graph_type_combo_box.setCurrentIndex(0)
        self._widget.graph_type_combo_box.currentIndexChanged.connect(self._refresh_rospackgraph)

        
        completionmodel = StackageCompletionModel(self._widget.filter_line_edit)
        completer = RepeatedWordCompleter(completionmodel, self)
        completer.setCompletionMode(QCompleter.PopupCompletion)
        completer.setWrapAround(True)

        completer.setCaseSensitivity(Qt.CaseInsensitive);
        self._widget.filter_line_edit.editingFinished.connect(self._refresh_rospackgraph)
        self._widget.filter_line_edit.setCompleter(completer)
        
        self._widget.with_stacks.clicked.connect(self._refresh_rospackgraph)
        self._widget.transitives.clicked.connect(self._refresh_rospackgraph)

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

        self._update_rospackgraph()
        self._deferred_fit_in_view.connect(self._fit_in_view, Qt.QueuedConnection)
        self._deferred_fit_in_view.emit()

        context.add_widget(self._widget)

    def save_settings(self, global_settings, perspective_settings):
        perspective_settings.set_value('graph_type_combo_box_index', self._widget.graph_type_combo_box.currentIndex())
        perspective_settings.set_value('filter_line_edit_text', self._widget.filter_line_edit.text())
        perspective_settings.set_value('with_stacks_state', self._widget.with_stacks.isChecked())
        perspective_settings.set_value('transitives_state', self._widget.transitives.isChecked())
        perspective_settings.set_value('auto_fit_graph_check_box_state', self._widget.auto_fit_graph_check_box.isChecked())
        perspective_settings.set_value('highlight_connections_check_box_state', self._widget.highlight_connections_check_box.isChecked())

    def restore_settings(self, global_settings, perspective_settings):
        self._widget.graph_type_combo_box.setCurrentIndex(int(perspective_settings.value('graph_type_combo_box_index', 0)))
        self._widget.filter_line_edit.setText(perspective_settings.value('filter_line_edit_text', ''))
        self._widget.with_stacks.setChecked(perspective_settings.value('with_stacks_state', True) in [True, 'true'])
        self._widget.transitives.setChecked(perspective_settings.value('transitives_state', True) in [True, 'true'])
        self._widget.auto_fit_graph_check_box.setChecked(perspective_settings.value('auto_fit_graph_check_box_state', True) in [True, 'true'])
        self._widget.highlight_connections_check_box.setChecked(perspective_settings.value('highlight_connections_check_box_state', True) in [True, 'true'])
        self._refresh_rospackgraph()

    def _update_rospackgraph(self):
        # re-enable controls customizing fetched ROS graph
        self._widget.graph_type_combo_box.setEnabled(True)
        self._widget.filter_line_edit.setEnabled(True)
        self._widget.with_stacks.setEnabled(True)
        self._widget.transitives.setEnabled(True)

        # self._graph = rospackgraph.impl.graph.Graph()
        # self._graph.set_master_stale(5.0)
        # self._graph.set_node_stale(5.0)
        # self._graph.update()
        self._refresh_rospackgraph()

    def _refresh_rospackgraph(self):
        self._update_graph_view(self._generate_dotcode())

    def _generate_dotcode(self):
        names = self._widget.filter_line_edit.text().split(',')
        includes = [x.strip() for x in names if not x.startswith('-')]
        excludes = [x.strip().lstrip('-') for x in names if x.startswith('-')]
        
        depth = self._widget.graph_type_combo_box.itemData(self._widget.graph_type_combo_box.currentIndex())
        # orientation = 'LR'
        return generate_dotcode(selected_names = includes,
                                excludes = excludes,
                                depth = depth,
                                with_stacks = self._widget.with_stacks.isChecked(),
                                hide_transitives = self._widget.transitives.isChecked(),
                                interstack_edges = True)
        #return generate_dotcode(self._graph, ns_filter, graph_mode, orientation, quiet)

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

        # layout graph
        graph = pydot.graph_from_dot_data(self._current_dotcode.encode("ascii","ignore"))

        #graph = pygraphviz.AGraph(string=self._current_dotcode, strict=False, directed=True)
        #graph.layout(prog='dot')


        # let pydot imitate pygraphviz api
        graph.nodes_iter = graph.get_node_list
        graph.edges_iter = graph.get_edge_list
        graph.subgraphs_iter = graph.get_subgraph_list

        nodes = {}
        for subgraph in graph.subgraphs_iter():
            stacknodeitem = getNodeItemForStack(subgraph, highlight_level)
            subgraph.nodes_iter = subgraph.get_node_list

            nodes[subgraph.get_name()] = stacknodeitem
            for node in subgraph.nodes_iter():
                # hack required by pydot
                if node.get_name() in ('graph', 'node', 'empty'):
                    continue
                nodes[node.get_name()] = getNodeItemForNode(node, highlight_level)
        for node in graph.nodes_iter():
            # hack required by pydot
            if node.get_name() in ('graph', 'node', 'empty'):
                continue
            nodes[node.get_name()] = getNodeItemForNode(node, highlight_level)

        edges = {} # is irrelevant?
        for subgraph in graph.subgraphs_iter():
            subgraph.edges_iter = subgraph.get_edge_list
            for edge in subgraph.edges_iter():
                edge_item = getEdgeItem(edge, nodes, edges, highlight_level)
                edge_item.add_to_scene(self._scene)
        for edge in graph.edges_iter():
            edge_item = getEdgeItem(edge, nodes, edges, highlight_level)
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
        self._widget.with_stacks.setEnabled(False)
        self._widget.transitives.setEnabled(False)

        self._update_graph_view(dotcode)

    def _fit_in_view(self):
        self._widget.graphics_view.fitInView(self._scene.itemsBoundingRect(), Qt.KeepAspectRatio)

    def _save_dot(self):
        file_name, _ = QFileDialog.getSaveFileName(self._widget, self.tr('Save as DOT'), 'rospackgraph.dot', self.tr('DOT graph (*.dot)'))
        if file_name is None or file_name == '':
            return

        file = QFile(file_name)
        if not file.open(QIODevice.WriteOnly | QIODevice.Text):
            return

        file.write(self._current_dotcode)
        file.close()

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
