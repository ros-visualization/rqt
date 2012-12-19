cmake_minimum_required(VERSION 2.8.3)
project(rqt_runtime_monitor)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS rospy diagnostic_msgs rqt_gui rqt_gui_py rospy roslib)
catkin_package()
catkin_python_setup()

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
