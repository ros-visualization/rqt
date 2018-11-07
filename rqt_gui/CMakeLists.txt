cmake_minimum_required(VERSION 3.5)
project(rqt_gui)

# Load catkin and all dependencies required for this package
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR src/${PROJECT_NAME})

install(PROGRAMS bin/rqt_gui
  DESTINATION bin
)
install(DIRECTORY resource
  DESTINATION share
)

ament_package()
