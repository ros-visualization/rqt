cmake_minimum_required(VERSION 2.8.3)
project(rqt_publisher)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS rqt_gui rqt_gui_py rqt_py_common qt_gui_py_common rosmsg)
catkin_package()
catkin_python_setup()

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
