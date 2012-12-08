find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

set(rqt_rviz_SRCS
  rviz.cpp
)

set(rqt_rviz_HDRS
  rviz.h
)

set(rqt_rviz_INCLUDE_DIRECTORIES
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/build/src
)

qt4_wrap_cpp(rqt_rviz_MOCS ${rqt_rviz_HDRS})

include_directories(${PROJECT_NAME} ${rqt_rviz_INCLUDE_DIRECTORIES})
rosbuild_add_library(${PROJECT_NAME} ${rqt_rviz_SRCS} ${rqt_rviz_MOCS})
target_link_libraries(${PROJECT_NAME} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})

find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})
