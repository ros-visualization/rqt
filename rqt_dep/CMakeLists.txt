cmake_minimum_required(VERSION 2.8.3)
project(rqt_dep)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS qt_dotgraph rqt_gui rqt_graph qt_gui_py_common python-rospkg)
catkin_package()
catkin_python_setup()

catkin_add_nosetests(test/dotcode_pack_test.py)