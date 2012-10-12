import roslib;roslib.load_manifest('nav_view')
import rospy
import tf

import random

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PolygonStamped, PointStamped

from QtCore import pyqtSignal, QPointF
from QtGui import QWidget, QPixmap, QImage, QGraphicsView, QGraphicsScene, QPainterPath, QPen, QColor, QPolygonF, QPushButton, QVBoxLayout, QHBoxLayout

from PIL import Image
from PIL.ImageQt import ImageQt

class PathInfo(object):
    def __init__(self, name=None):
        self.color = None
        self.sub = None
        self.cb = None
        self.path = None
        self.item = None

        self.name = name

class NavViewWidget(QWidget):
    def __init__(self, map_topic = '/map', 
                 paths = ['/move_base/SBPLLatticePlanner/plan', '/move_base/TrajectoryPlannerROS/local_plan'], 
                 polygons= ['/move_base/local_costmap/robot_footprint']):
        super(NavViewWidget, self).__init__()
        self._layout = QVBoxLayout()

        self.setWindowTitle('Navigation Viewer')
        self._nav_view = NavView(map_topic, paths, polygons)
        self._layout.addWidget(self._nav_view)

        self.setLayout(self._layout)

class NavView(QGraphicsView):
    map_changed = pyqtSignal()
    path_changed = pyqtSignal(str)
    polygon_changed = pyqtSignal(str)
    def __init__(self, map_topic = '/map', 
                 paths = ['/move_base/SBPLLatticePlanner/plan', '/move_base/TrajectoryPlannerROS/local_plan'], 
                 polygons= ['/move_base/local_costmap/robot_footprint']):
        super(QWidget, self).__init__()

        self.map_changed.connect(self._update)
        self.destroyed.connect(self.close)
        
        #ScrollHandDrag
        self.setDragMode(1)

        self._map = None
        self._map_orig = None
        self._map_item = None

        self.w = 0
        self.h = 0

        self._paths = {}
        self._polygons = {}
        self.path_changed.connect(self._update_path)
        self.polygon_changed.connect(self._update_polygon)

        self._colors = [(238, 34, 116), (68, 134, 252), (236, 228, 46), (102, 224, 18), (242, 156, 6), (240, 64, 10), (196, 30, 250)]

        self._scene = QGraphicsScene()

        self._tf = tf.TransformListener()
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_cb) 

        for path in paths:
            self.add_path(path)

        for poly in polygons:
            self.add_polygon(poly)

        self.setScene(self._scene)

    def wheelEvent(self, event):
        event.ignore()
        
        if event.delta() > 0:
            self.zoom_in(2)
        else:
            self.zoom_out(2)

    def map_cb(self, msg):
        self.resolution = msg.info.resolution
        self.w = msg.info.width
        self.h = msg.info.height

        data = []

        # Get correct colors
        for c in msg.data:
            if c == 100:
                data.append((0, 0, 0))
            elif c == 0:
                data.append((255, 255, 255))
            else:
                data.append((127, 127, 127))

        im = Image.new('RGB', (self.w, self.h))
        im.putdata(data)
        self._map = im
        self._map_orig = self._map

        self.map_changed.emit()

    def zoom_in(self, amount):
        self.w = self.w * amount
        self.h = self.h * amount
        self._map = self._map_orig.resize((self.w, self.h))
        self.resolution = self.resolution / amount
        self.map_changed.emit()

    def zoom_out(self, amount):
        self.w = self.w / amount
        self.h = self.h / amount
        self._map = self._map_orig.resize((self.w, self.h))
        self.resolution = self.resolution * amount
        self.map_changed.emit()

    def add_path(self, name):
        path = PathInfo(name)
        def c(msg):
            pp = QPainterPath()

            # Transform everything in to the map frame
            if not (msg.header.frame_id == '/map' or msg.header.frame_id == ''):
                try:
                    self._tf.waitForTransform(msg.header.frame_id, '/map', rospy.Time(), rospy.Duration(10))
                    data = [self._tf.transformPose('/map', pose) for pose in msg.poses]
                except tf.Exception as e:
                    rospy.logerr("TF Error")
                    data = []
            else:
                data = msg.poses

            if len(data) > 0:
                start = data[0].pose.position
                pp.moveTo(start.x / self.resolution, start.y / self.resolution)

                for pose in data:
                    pt = pose.pose.position
                    pp.lineTo(pt.x / self.resolution, pt.y / self.resolution)

                path.path = pp
                self.path_changed.emit(name)

        path.color = random.choice(self._colors)
        self._colors.remove(path.color)

        path.cb = c
        path.sub = rospy.Subscriber(path.name, Path, path.cb)

        self._paths[name] = path

    def add_polygon(self, name):
        poly = PathInfo(name)
        def c(msg):
            if not (msg.header.frame_id == '/map' or msg.header.frame_id == ''):
                try:
                    self._tf.waitForTransform(msg.header.frame_id, '/map', rospy.Time(), rospy.Duration(10))
                    points_stamped = []
                    for pt in msg.polygon.points:
                        ps = PointStamped()
                        ps.header.frame_id = msg.header.frame_id
                        ps.point.x = pt.x
                        ps.point.y = pt.y

                        points_stamped.append(ps)

                    trans_pts = [self._tf.transformPoint('/map', pt).point for pt in points_stamped]
                except tf.Exception:
                    rospy.logerr("TF Error")
                    trans_pts = []
            else:
                trans_pts = [pt for pt in msg.polygon.points]

            if len(trans_pts) > 0:
                pts = [QPointF(pt.x / self.resolution, pt.y / self.resolution) for pt in trans_pts]

                close = trans_pts[0]
                pts.append(QPointF(close.x / self.resolution, close.y / self.resolution))
                poly.path = QPolygonF(pts)

                self.polygon_changed.emit(name)

        poly.color = random.choice(self._colors)
        self._colors.remove(poly.color)

        poly.cb = c
        poly.sub = rospy.Subscriber(poly.name, PolygonStamped, poly.cb)

        self._polygons[name] = poly

    def close(self):
        if self.map_sub:
            self.map_sub.unregister()

        for p in self._paths:
            if p.sub:
                p.sub.unregister()

        for p in self._polygons:
            if p.sub:
                p.sub.unregister()

    def _update(self):
        if self._map_item:
            self._scene.removeItem(self._map_item)

        self.pix = ImageQt(self._map)
        self.setSceneRect(0,0, self.w, self.h)
        self._map_item = self._scene.addPixmap(QPixmap.fromImage(self.pix))

        # Everything must be mirrored
        self._mirror(self._map_item)

        self.centerOn(self._map_item)
        self.show()

    def _update_path(self, name):
        old_item = None
        if name in self._paths.keys():
            old_item = self._paths[name].item

        self._paths[name].item = self._scene.addPath(self._paths[name].path, pen = QPen(QColor(*self._paths[name].color)))

        # Everything must be mirrored
        self._mirror(self._paths[name].item)

        if old_item:
            self._scene.removeItem(old_item)

    def _update_polygon(self, name):
        old_item = None
        if name in self._polygons.keys():
            old_item = self._polygons[name].item

        self._polygons[name].item = self._scene.addPolygon(self._polygons[name].path, pen = QPen(QColor(*self._polygons[name].color)))

        # Everything must be mirrored
        self._mirror(self._polygons[name].item)

        if old_item:
            self._scene.removeItem(old_item)

    def _mirror(self, item):
        item.scale(-1,1)
        item.translate(-self.w, 0)

