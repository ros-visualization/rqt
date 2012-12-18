cmake_minimum_required(VERSION 2.8.3)
project(rqt_pose_view)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS rqt_gui rqt_gui_py rostopic geometry_msgs tf python-opengl python-qt-bindings-gl)
catkin_package()
catkin_python_setup()