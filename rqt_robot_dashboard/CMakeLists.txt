cmake_minimum_required(VERSION 2.8.3)
project(rqt_robot_dashboard)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS rospy rqt_gui rqt_gui_py rqt_robot_monitor rqt_console rqt_nav_view)
catkin_package()
catkin_python_setup()