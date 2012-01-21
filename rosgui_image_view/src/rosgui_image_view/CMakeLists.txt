find_package(Qt4 4.6 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

set(rosgui_image_view_SRCS
  image_view.cpp
  ratio_layouted_frame.cpp
)

set(rosgui_image_view_HDRS
  image_view.h
  ratio_layouted_frame.h
)

set(rosgui_image_view_UIS
  image_view.ui
)

set(rosgui_image_view_INCLUDE_DIRECTORIES
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/build/src
)

qt4_wrap_cpp(rosgui_image_view_MOCS ${rosgui_image_view_HDRS})
qt4_wrap_ui(rosgui_image_view_UIS_H ${rosgui_image_view_UIS})

string(REGEX REPLACE "^([^;]*;)*([^;]+/rosgui_cpp)/include(;[^;]*)*$" "\\2" rosgui_cpp_SOURCE_DIR "${rosgui_image_view_INCLUDE_DIRS}")
include(${rosgui_cpp_SOURCE_DIR}/cmake/hide_plugin_symbols.cmake)
rosgui_cpp__hide_plugin_symbols(rosgui_image_view ImageView)

include_directories(${PROJECT_NAME} ${rosgui_image_view_INCLUDE_DIRECTORIES})
rosbuild_add_library(${PROJECT_NAME} ${rosgui_image_view_SRCS} ${rosgui_image_view_MOCS} ${rosgui_image_view_UIS_H})
target_link_libraries(${PROJECT_NAME} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})

