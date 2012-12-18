cmake_minimum_required(VERSION 2.8.3)
project(rqt_gui_cpp)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS qt_gui qt_gui_cpp roscpp nodelet)
catkin_package()
catkin_python_setup()

#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_subdirectory(src)

