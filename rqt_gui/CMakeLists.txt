cmake_minimum_required(VERSION 2.8.3)
project(rqt_gui)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS qt_gui)
catkin_package()
catkin_python_setup()

install(PROGRAMS bin/rqt_gui
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(FILES resource/rqt.png
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/resource
)
