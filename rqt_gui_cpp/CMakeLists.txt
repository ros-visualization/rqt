cmake_minimum_required(VERSION 3.0.2)
project(rqt_gui_cpp)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS qt_gui qt_gui_cpp roscpp nodelet)
catkin_package(
	INCLUDE_DIRS include
	LIBRARIES ${PROJECT_NAME}
	CATKIN_DEPENDS qt_gui qt_gui_cpp roscpp nodelet
)

include_directories(include ${catkin_INCLUDE_DIRS})

find_package(Qt5Widgets REQUIRED)

SET(rqt_gui_cpp_SRCS
  src/rqt_gui_cpp/nodelet_plugin_provider.cpp
  src/rqt_gui_cpp/roscpp_plugin_provider.cpp
)

set(rqt_gui_cpp_HDRS
  src/rqt_gui_cpp/roscpp_plugin_provider.h
)

qt5_wrap_cpp(rqt_gui_cpp_MOCS ${rqt_gui_cpp_HDRS})

add_library(${PROJECT_NAME} ${rqt_gui_cpp_SRCS} ${rqt_gui_cpp_MOCS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} Qt5::Widgets)

if(APPLE)
  set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS "-undefined dynamic_lookup")
endif()

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
