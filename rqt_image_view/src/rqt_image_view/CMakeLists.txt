find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

set(rqt_image_view_SRCS
  image_view.cpp
  ratio_layouted_frame.cpp
)

set(rqt_image_view_HDRS
  image_view.h
  ratio_layouted_frame.h
)

set(rqt_image_view_UIS
  image_view.ui
)

set(rqt_image_view_INCLUDE_DIRECTORIES
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/build/src
)

qt4_wrap_cpp(rqt_image_view_MOCS ${rqt_image_view_HDRS})
qt4_wrap_ui(rqt_image_view_UIS_H ${rqt_image_view_UIS})

include_directories(${PROJECT_NAME} ${rqt_image_view_INCLUDE_DIRECTORIES})
rosbuild_add_library(${PROJECT_NAME} ${rqt_image_view_SRCS} ${rqt_image_view_MOCS} ${rqt_image_view_UIS_H})
target_link_libraries(${PROJECT_NAME} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})

find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})
