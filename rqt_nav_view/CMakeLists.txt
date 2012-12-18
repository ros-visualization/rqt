cmake_minimum_required(VERSION 2.8.3)
project(rqt_nav_view)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS  rospy rqt_gui rqt_gui_py tf nav_msgs geometry_msgs)
catkin_package()
catkin_python_setup()
