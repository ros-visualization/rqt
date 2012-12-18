cmake_minimum_required(VERSION 2.8.3)
project(rqt_tf_tree)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS rqt_gui qt_dotgraph rqt_graph rospy geometry_msgs tf2)
catkin_package()
catkin_python_setup()