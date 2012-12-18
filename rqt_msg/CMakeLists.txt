cmake_minimum_required(VERSION 2.8.3)
project(rqt_msg)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS rospy rqt_gui rqt_gui_py rqt_py_common rosmsg roslib rqt_console)
catkin_package()
catkin_python_setup()

