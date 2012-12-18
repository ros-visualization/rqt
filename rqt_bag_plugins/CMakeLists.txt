cmake_minimum_required(VERSION 2.8.3)
project(rqt_bag_plugins)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS rospy rqt_gui rqt_gui_py roslib std_msgs rosbag rqt_bag geometry_msgs sensor_msgs)
catkin_package()
catkin_python_setup()

