cmake_minimum_required(VERSION 3.0.2)
project(rqt_gui)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS qt_gui)
catkin_package()
catkin_python_setup()

catkin_install_python(PROGRAMS bin/rqt_gui
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(FILES resource/rqt.png
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/resource
)
