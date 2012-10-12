cmake_minimum_required(VERSION 2.8)
project(robot_dashboard)

find_package(catkin REQUIRED)
catkin_stack()
catkin_project(robot_dashboard)

# Find the setup.py
catkin_python_setup()

# Build the docs
#catkin_sphinx("doc/source" "doc/build")
