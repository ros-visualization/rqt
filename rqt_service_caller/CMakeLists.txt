cmake_minimum_required(VERSION 2.8.3)
project(rqt_service_caller)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS rqt_gui rqt_gui_py rqt_py_common rosservice)
catkin_package()
catkin_python_setup()