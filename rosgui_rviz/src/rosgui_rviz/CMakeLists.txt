find_package(Qt4 4.6 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

set(rosgui_rviz_SRCS
  rviz.cpp
)

set(rosgui_rviz_HDRS
  rviz.h
)

set(rosgui_rviz_INCLUDE_DIRECTORIES
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/build/src
)

qt4_wrap_cpp(rosgui_rviz_MOCS ${rosgui_rviz_HDRS})

string(REGEX REPLACE "^([^;]*;)*([^;]+/rosgui_cpp)/include(;[^;]*)*$" "\\2" rosgui_cpp_SOURCE_DIR "${rosgui_rviz_INCLUDE_DIRS}")
include(${rosgui_cpp_SOURCE_DIR}/cmake/hide_plugin_symbols.cmake)
rosgui_cpp__hide_plugin_symbols(rosgui_rviz RViz)

include_directories(${PROJECT_NAME} ${rosgui_rviz_INCLUDE_DIRECTORIES})
rosbuild_add_library(${PROJECT_NAME} ${rosgui_rviz_SRCS} ${rosgui_rviz_MOCS})
target_link_libraries(${PROJECT_NAME} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
