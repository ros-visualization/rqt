cmake_minimum_required(VERSION 2.8.3)
project(rqt_rviz)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS rqt_gui rqt_gui_cpp rviz)
catkin_package(	
	INCLUDE_DIRS ${rqt_rviz_INCLUDE_DIRECTORIES}
	LIBRARIES ${PROJECT_NAME} 
	CATKIN_DEPENDS rqt_gui rqt_gui_cpp rviz
)
catkin_python_setup()

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

set(rqt_rviz_SRCS
  src/rqt_rviz/rviz.cpp
)

set(rqt_rviz_HDRS
  include/rqt_rviz/rviz.h
)

qt4_wrap_cpp(rqt_rviz_MOCS ${rqt_rviz_HDRS})

include_directories(include ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME} ${rqt_rviz_SRCS} ${rqt_rviz_MOCS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})

find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(PROGRAMS scripts/rqt_rviz
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
