Changelog for package rqt_gui
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.2 (2020-06-12)
------------------
* add missing dependencies: rospkg-modules, python_qt_binding, rospy (`#227 <https://github.com/ros-visualization/rqt/issues/227>`_)

0.5.1 (2020-03-10)
------------------
* bump CMake minimum version to avoid CMP0048 warning (`#219 <https://github.com/ros-visualization/rqt/issues/219>`_)
* allow definition of settings file (`#216 <https://github.com/ros-visualization/rqt/issues/216>`_)
* use catkin_install_python for Python script (`#206 <https://github.com/ros-visualization/rqt/issues/206>`_)
* style changes (`#143 <https://github.com/ros-visualization/rqt/issues/143>`_)
* autopep8 (`#137 <https://github.com/ros-visualization/rqt/issues/137>`_)

0.5.0 (2017-04-24)
------------------
* version bump to match version of migrated package `rqt_py_common`

0.3.2 (2017-01-24)
------------------
* use Python 3 compatible syntax (`#113 <https://github.com/ros-visualization/rqt/pull/113>`_)

0.3.1 (2016-04-01)
------------------

0.3.0 (2016-04-01)
------------------
* switch to Qt5 (`#106 <https://github.com/ros-visualization/rqt/pull/106>`_)

0.2.14 (2014-03-04)
-------------------

0.2.13 (2014-01-08)
-------------------
* added hash of ros package path as prefix for plugin manager settings to allow for one cache per package path setting

0.2.12 (2013-10-09)
-------------------
* improve startup time (`#88 <https://github.com/ros-visualization/rqt/issues/88>`_)
* new icon for rqt

0.2.11 (2013-09-06)
-------------------

0.2.10 (2013-08-21)
-------------------
* add application icon (`#82 <https://github.com/ros-visualization/rqt/issues/82>`_)
* improve error message when plugin xml file does not exist

0.2.9 (2013-06-06)
------------------
* make plugin resources relative to plugin.xml (`ros-visualization/qt_gui_core#16 <https://github.com/ros-visualization/qt_gui_core/issues/16>`_)
* use standard rospy function to filter remapping arguments (`#76 <https://github.com/ros-visualization/rqt/issues/76>`_)
* fix help provider

0.2.8 (2013-01-11)
------------------
* properly ignore ROS remapping arguments when passed in via command line

0.2.7 (2012-12-31)
------------------
* first public release for Groovy
