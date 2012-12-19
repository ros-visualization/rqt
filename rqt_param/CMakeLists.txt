cmake_minimum_required(VERSION 2.8.3)
project(rqt_param)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS rospy dynamic_reconfigure qt_gui rqt_gui rqt_gui_py rqt_py_common)
catkin_package()
catkin_python_setup()

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
