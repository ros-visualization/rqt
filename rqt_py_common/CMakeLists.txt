cmake_minimum_required(VERSION 2.8.3)
project(rqt_py_common)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS qt_gui rospy rostopic)
catkin_package()
catkin_python_setup()
