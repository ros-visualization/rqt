cmake_minimum_required(VERSION 2.8.3)
project(rqt_console)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS rospy rqt_gui rqt_gui_py roslib)
catkin_package()
catkin_python_setup()

