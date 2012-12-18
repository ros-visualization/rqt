cmake_minimum_required(VERSION 2.8.3)
project(rqt_graph)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS rqt_gui qt_dotgraph rosgraph roslib rosnode rostopic rosservice)
catkin_package()
catkin_python_setup()

catkin_add_nosetests(test/dotcode_test.py)