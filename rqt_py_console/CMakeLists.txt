cmake_minimum_required(VERSION 2.8.3)
project(rqt_py_console)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS qt_gui qt_gui_py_common rqt_gui rqt_gui_py rospy)
catkin_package()
catkin_python_setup()