Changelog for package rqt_gui
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2
===

0.2.8 (2013-01-11)
------------------
* properly ignore ROS remapping arguments when passed in via command line

0.2.7 (2012-12-31)
------------------
* readd rqt_gui script which enables calling `rosrun rqt_gui rqt_gui`

0.2.6 (2012-12-23)
------------------
* no changes, released along with other packages

0.2.5 (2012-12-21)
------------------
* add run dependency on catkin

0.2.4 (2012-12-21)
------------------
* no changes, released along with other packages

0.2.3 (2012-12-21)
------------------
* no changes, released along with other packages

0.2.2 (2012-12-20)
------------------
* no changes, released along with other packages

0.2.1 (2012-12-20)
------------------
* no changes, released along with other packages

0.2.0 (2012-12-20)
------------------
* split framework and plugins into separate repositories
* convert buildsystem from rosbuild to catkin
* change import style for Qt modules to use python_qt_binding directly: `from python_qt_binding.QtCore import QObject`
* install global `rqt` script
* add plugin extension point to discover ROS-agnostic plugins

0.1
===

0.1.2 (2012-08-10)
------------------
* new plugins:
  * rqt_bag: plugin to load, introspect and playback ROS bag files
* rqt_deb, rqt_publisher: use the WorkerThread class
* rqt_robot_steering: fix stopping timer on shutdown
* rqt_web: add dropdown completer
* updated dependencies to match new rosdeps

0.1.1 (2012-07-30)
------------------
* new plugins:
  * rqt_py_console: a simple interactive python console plugin
  * rqt_web: WebKit plugin for displaying and interacting with web tools
* rqt_console: fix default value of show_highlighted_only

0.1.0 (2012-07-18)
------------------
* first released version

