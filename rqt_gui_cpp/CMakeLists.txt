cmake_minimum_required(VERSION 2.8.3)
project(rqt_gui_cpp)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS qt_gui qt_gui_cpp roscpp nodelet)
catkin_package(
	INCLUDE_DIRS include 
	LIBRARIES ${PROJECT_NAME} 
	CATKIN_DEPENDS qt_gui qt_gui_cpp roscpp nodelet
)

include_directories(include)

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

SET(rqt_gui_cpp_SRCS
  src/rqt_gui_cpp/nodelet_plugin_provider.cpp
  src/rqt_gui_cpp/roscpp_plugin_provider.cpp
)

add_library(${PROJECT_NAME} ${rqt_gui_cpp_SRCS})
target_link_libraries(${PROJECT_NAME} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})

if(APPLE) 
  set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS "-undefined dynamic_lookup")
endif()

