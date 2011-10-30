from __future__ import division
import os

# pydot requires some hacks
import pydot
# TODO: use pygraphviz instead, but non-deterministic layout will first be resolved in graphviz 2.30
#import pygraphviz

from rosgui.QtBindingHelper import loadUi
from QtCore import QEvent, QFile, QIODevice, QObject, QPointF, QRectF, Qt, QTextStream, Signal
from QtGui import QDockWidget, QFileDialog, QGraphicsScene, QGraphicsView, QIcon, QImage, QPainter, QTransform
from QtSvg import QSvgGenerator

import roslib
roslib.load_manifest('rosgui_rosgraph')
import rosgraph.impl.graph

from dotcode import generate_dotcode, NODE_NODE_GRAPH, NODE_TOPIC_ALL_GRAPH, NODE_TOPIC_GRAPH
from EdgeItem import EdgeItem
from NodeItem import NodeItem

class RosGraph(QObject):

    _deferred_fit_in_view = Signal()

    def __init__(self, parent, plugin_context):
        super(RosGraph, self).__init__(parent)
        self.setObjectName('RosGraph')

        self._graph = None
        self._current_dotcode = None
        self._last_pan_point = None
        self._last_scene_center = None

        main_window = plugin_context.main_window()
        self._widget = QDockWidget(main_window)

        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'RosGraph.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('RosGraphUi')
        if plugin_context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % plugin_context.serial_number()))
        main_window.addDockWidget(Qt.RightDockWidgetArea, self._widget)

        self._scene = QGraphicsScene()
        self._widget.graphics_view.setScene(self._scene)
        self._widget.graphics_view.mousePressEvent = self._graphics_view_mousePressEvent
        self._widget.graphics_view.mouseReleaseEvent = self._graphics_view_mouseReleaseEvent
        self._widget.graphics_view.mouseMoveEvent = self._graphics_view_mouseMoveEvent
        self._widget.graphics_view.wheelEvent = self._graphics_view_wheelEvent

        self._widget.graph_type_combo_box.insertItem(0, self.tr('Node/Node Connectivity'), NODE_NODE_GRAPH)
        self._widget.graph_type_combo_box.insertItem(1, self.tr('Node/Topic Connections'), NODE_TOPIC_GRAPH)
        self._widget.graph_type_combo_box.insertItem(2, self.tr('Node/Topic Connections (only with active network connections)'), NODE_TOPIC_ALL_GRAPH)
        self._widget.graph_type_combo_box.setCurrentIndex(0)
        self._widget.graph_type_combo_box.currentIndexChanged.connect(self._refresh_rosgraph)
        self._widget.filter_line_edit.editingFinished.connect(self._refresh_rosgraph)
        self._widget.quiet_check_box.clicked.connect(self._refresh_rosgraph)

        self._widget.refresh_graph_push_button.setIcon(QIcon.fromTheme('view-refresh'))
        self._widget.refresh_graph_push_button.pressed.connect(self._update_rosgraph)

        self._widget.highlight_level_spin_box.valueChanged.connect(self._refresh_graph)
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

        # trigger deleteLater for plugin when _widget is closed
        self._widget.installEventFilter(self)

    def _update_rosgraph(self):
        # re-enable controls customizing fetched ROS graph
        self._widget.graph_type_combo_box.setEnabled(True)
        self._widget.filter_line_edit.setEnabled(True)
        self._widget.quiet_check_box.setEnabled(True)

        if self._graph is None:
            node_ns = None
            topic_ns = None
            self._graph = rosgraph.impl.graph.Graph(node_ns, topic_ns)
            self._graph.set_master_stale(5.0)
            self._graph.set_node_stale(5.0)
        self._graph.update()
        self._refresh_rosgraph()

    def _refresh_rosgraph(self):
        self._update_graph(self._generate_dotcode())

    def _generate_dotcode(self):
        ns_filter = self._widget.filter_line_edit.text()
        if ns_filter == '' or ns_filter[-1] != '/':
            ns_filter += '/'
        graph_mode = self._widget.graph_type_combo_box.itemData(self._widget.graph_type_combo_box.currentIndex())
        orientation = 'LR'
        quiet = self._widget.quiet_check_box.isChecked()
        return generate_dotcode(self._graph, ns_filter, graph_mode, orientation, quiet)

    def _update_graph(self, dotcode):
        if dotcode == self._current_dotcode:
            return
        self._current_dotcode = dotcode
        self._refresh_graph()

    def _refresh_graph(self):
        self._scene.clear()

        highlight_level = self._widget.highlight_level_spin_box.value()

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
            if node.get_name() == 'graph' or node.get_name() == 'node':
                continue
            # let pydot imitate pygraphviz api
            attr = {}
            for name in node.get_attributes().iterkeys():
                value = get_unquoted(node, name)
                attr[name] = value
            node.attr = attr

            # decrease rect by one so that edges do not reach inland
            ellipse_rect = QRectF(0, 0, POINTS_PER_INCH * float(node.attr['width']) - 1.0, POINTS_PER_INCH * float(node.attr['height']) - 1.0)
            pos = node.attr['pos'].split(',')
            ellipse_rect.moveCenter(QPointF(float(pos[0]), -float(pos[1])))
            node_item = NodeItem(highlight_level, ellipse_rect, node.attr['label'])

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

            label = edge.attr['label']
            pos = edge.attr['lp']
            label_center = None
            if pos is not None:
                pos = pos.split(',')
                label_center = QPointF(float(pos[0]), -float(pos[1]))

            # try pydot, fallback for pygraphviz
            source_node = edge.get_source() if hasattr(edge, 'get_source') else edge[0]
            destination_node = edge.get_destination() if hasattr(edge, 'get_destination') else edge[1]

            # create edge with from-node and to-node
            edge_item = EdgeItem(highlight_level, edge.attr['pos'], label_center, label, nodes[source_node], nodes[destination_node])
            # symmetrically add all sibling edges with same label
            if not edges.has_key(label):
                edges[label] = []
            for sibling in edges[label]:
                edge_item.add_sibling_edge(sibling)
                sibling.add_sibling_edge(edge_item)
            edges[label].append(edge_item)
            edge_item.add_to_scene(self._scene)

        for node_item in nodes.itervalues():
            self._scene.addItem(node_item)

        self._scene.setSceneRect(self._scene.itemsBoundingRect())

    def _load_dot(self, file_name=None):
        if file_name is None:
            file_name = QFileDialog.getOpenFileName(self._widget, self.tr('Open graph from file'), None, self.tr('DOT graph (*.dot)'))
            # return type might be a string or a tuple
            try:
                file_name, _ = file_name
            except ValueError:
                pass
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

        self._update_graph(dotcode)

    def _fit_in_view(self):
        self._widget.graphics_view.fitInView(self._scene.itemsBoundingRect(), Qt.KeepAspectRatio)

    def _save_dot(self):
        file_name = QFileDialog.getSaveFileName (self._widget, self.tr('Save as DOT'), 'rosgraph.dot', self.tr('DOT graph (*.dot)'))
        # return type might be a string or a tuple
        try:
            file_name, _ = file_name
        except ValueError:
            pass
        if file_name is None or file_name == '':
            return

        file = QFile(file_name)
        if not file.open(QIODevice.WriteOnly | QIODevice.Text):
            return

        file.write(self._current_dotcode)
        file.close()

    def _save_svg(self):
        file_name = QFileDialog.getSaveFileName (self._widget, self.tr('Save as SVG'), 'rosgraph.svg', self.tr('Scalable Vector Graphic (*.svg)'))
        # return type might be a string or a tuple
        try:
            file_name, _ = file_name
        except ValueError:
            pass
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
        file_name = QFileDialog.getSaveFileName (self._widget, self.tr('Save as image'), 'rosgraph.png', self.tr('Image (*.bmp *.jpg *.png *.tiff)'))
        # return type might be a string or a tuple
        try:
            file_name, _ = file_name
        except ValueError:
            pass
        if file_name is None or file_name == '':
            return

        img = QImage((self._scene.sceneRect().size() * 2.0).toSize(), QImage.Format_ARGB32_Premultiplied)
        painter = QPainter(img)
        painter.setRenderHint(QPainter.Antialiasing)
        self._scene.render(painter)
        painter.end()
        img.save(file_name)

    def _graphics_view_mousePressEvent(self, mouse_event):
        self._last_pan_point = mouse_event.pos()
        self._last_scene_center = self._map_to_scene_f(QRectF(self._widget.graphics_view.frameRect()).center())
        self._widget.graphics_view.setCursor(Qt.ClosedHandCursor)

    def _graphics_view_mouseReleaseEvent(self, mouse_event):
        self._widget.graphics_view.setCursor(Qt.OpenHandCursor)
        self._last_pan_point = None

    def _graphics_view_mouseMoveEvent(self, mouse_event):
        if self._last_pan_point is not None:
            delta_scene = self._widget.graphics_view.mapToScene(mouse_event.pos()) - self._widget.graphics_view.mapToScene(self._last_pan_point)
            if not delta_scene.isNull():
                self._widget.graphics_view.centerOn(self._last_scene_center - delta_scene)
                self._last_scene_center -= delta_scene
            self._last_pan_point = mouse_event.pos()
        QGraphicsView.mouseMoveEvent(self._widget.graphics_view, mouse_event)

    def _graphics_view_wheelEvent(self, wheel_event):
        if wheel_event.modifiers() == Qt.NoModifier:
            num_degrees = wheel_event.delta() / 8.0
            num_steps = num_degrees / 15.0
            mouse_before_scale_in_scene = self._widget.graphics_view.mapToScene(wheel_event.pos())

            scale_factor = 1.2 * num_steps
            if num_steps < 0:
                scale_factor = -1.0 / scale_factor
            scaling = QTransform(scale_factor, 0, 0, scale_factor, 0, 0)
            self._widget.graphics_view.setTransform(self._widget.graphics_view.transform() * scaling)

            mouse_after_scale_in_scene = self._widget.graphics_view.mapToScene(wheel_event.pos())
            center_in_scene = self._widget.graphics_view.mapToScene(self._widget.graphics_view.frameRect().center())
            self._widget.graphics_view.centerOn(center_in_scene + mouse_before_scale_in_scene - mouse_after_scale_in_scene)

            wheel_event.accept()
        else:
            QGraphicsView.wheelEvent(self._widget.graphics_view, wheel_event)

    def _map_to_scene_f(self, pointf):
        point = pointf.toPoint()
        if pointf.x() == point.x() and pointf.y() == point.y():
            # map integer coordinates
            return self._widget.graphics_view.mapToScene(point)
        elif pointf.x() == point.x():
            # map integer x and decimal y coordinates
            pointA = self._widget.graphics_view.mapToScene((pointf + QPointF(0, -0.5)).toPoint())
            pointB = self._widget.graphics_view.mapToScene((pointf + QPointF(0, 0.5)).toPoint())
            return (pointA + pointB) / 2.0
        elif pointf.y() == point.y():
            # map decimal x  and integer y and coordinates
            pointA = self._widget.graphics_view.mapToScene((pointf + QPointF(-0.5, 0)).toPoint())
            pointB = self._widget.graphics_view.mapToScene((pointf + QPointF(0.5, 0)).toPoint())
            return (pointA + pointB) / 2.0
        else:
            # map decimal coordinates
            pointA = self._widget.graphics_view.mapToScene((pointf + QPointF(-0.5, -0.5)).toPoint())
            pointB = self._widget.graphics_view.mapToScene((pointf + QPointF(-0.5, 0.5)).toPoint())
            pointC = self._widget.graphics_view.mapToScene((pointf + QPointF(0.5, -0.5)).toPoint())
            pointD = self._widget.graphics_view.mapToScene((pointf + QPointF(0.5, 0.5)).toPoint())
            return (pointA + pointB + pointC + pointD) / 4.0

    def eventFilter(self, obj, event):
        if obj is self._widget and event.type() == QEvent.Close:
            # TODO: ignore() should not be necessary when returning True
            event.ignore()
            self.deleteLater()
            return True
        return QObject.eventFilter(self, obj, event)

    def close_plugin(self):
        self._widget.close()
        self._widget.deleteLater()
